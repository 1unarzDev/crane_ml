using System;
using System.Diagnostics;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater {
    /// <summary>Frames from a RenderTexture to an ffmpeg process (raw RGBA on its stdin, libx264, one encoder
    /// thread so the bitstream is reproducible), with the sha256 of every frame's pixels. Without ffmpeg
    /// (path "none") every frame is written as a PNG instead.</summary>
    public sealed class TheaterCapture : IDisposable {
        readonly int w, h;
        readonly byte[] pixels;
        readonly Process ff;
        readonly Stream stdin;
        readonly StringBuilder ffLog = new StringBuilder();
        readonly string pngDir;
        readonly SHA256 sha = SHA256.Create();
        Texture2D readTex;
        public int Frames { get; private set; }
        public string Output { get; }
        public string FfmpegLog => ffLog.ToString();

        public TheaterCapture(string ffmpeg, string output, int width, int height, float fps, int crf, string preset) {
            w = width; h = height; Output = output;
            pixels = new byte[w * h * 4];
            if (string.IsNullOrEmpty(ffmpeg) || ffmpeg == "none") {
                pngDir = Path.Combine(Path.GetDirectoryName(output) ?? ".", "frames");
                Directory.CreateDirectory(pngDir);
                return;
            }
            string fr = fps.ToString(System.Globalization.CultureInfo.InvariantCulture);
            var psi = new ProcessStartInfo {
                FileName = ffmpeg,
                Arguments = $"-hide_banner -loglevel error -y -f rawvideo -pix_fmt rgba -s {w}x{h} -framerate {fr} -i - " +
                            $"-vf vflip -c:v libx264 -preset {preset} -crf {crf} -pix_fmt yuv420p -threads 1 -g {Mathf.RoundToInt(fps * 2)} " +
                            $"-movflags +faststart \"{output}\"",
                UseShellExecute = false, RedirectStandardInput = true, RedirectStandardError = true, RedirectStandardOutput = false,
                CreateNoWindow = true
            };
            ff = Process.Start(psi) ?? throw new InvalidOperationException($"theater: could not start {ffmpeg}");
            ff.ErrorDataReceived += (_, e) => { if (e.Data != null) lock (ffLog) ffLog.AppendLine(e.Data); };
            ff.BeginErrorReadLine();
            stdin = ff.StandardInput.BaseStream;
        }

        /// <summary>Reads the target back, sends it on, and returns the frame's sha256 (hex).</summary>
        public string Write(RenderTexture rt, int frameIndex) {
            var req = AsyncGPUReadback.Request(rt, 0, TextureFormat.RGBA32);
            req.WaitForCompletion();
            if (!req.hasError) req.GetData<byte>().CopyTo(pixels);
            else {
                readTex ??= new Texture2D(w, h, TextureFormat.RGBA32, false);
                var prev = RenderTexture.active;
                RenderTexture.active = rt;
                readTex.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);
                RenderTexture.active = prev;
                readTex.GetRawTextureData<byte>().CopyTo(pixels);
            }
            string hex = BitConverter.ToString(sha.ComputeHash(pixels)).Replace("-", "").ToLowerInvariant();
            if (ff != null) {
                if (ff.HasExited) throw new IOException($"ffmpeg exited ({ff.ExitCode}): {FfmpegLog}");
                stdin.Write(pixels, 0, pixels.Length);
            }
            else WritePng(Path.Combine(pngDir, $"frame_{frameIndex:D6}.png"));
            Frames++;
            return hex;
        }

        /// <summary>The last frame read back, as a PNG (stills for contact sheets and checks).</summary>
        public void WritePng(string path) {
            readTex ??= new Texture2D(w, h, TextureFormat.RGBA32, false);
            readTex.LoadRawTextureData(pixels);
            readTex.Apply(false);
            File.WriteAllBytes(path, readTex.EncodeToPNG());
        }

        /// <summary>Closes the encoder; true when it exited cleanly.</summary>
        public bool Finish(int timeoutMs = 600000) {
            if (ff == null) return true;
            try { stdin.Flush(); stdin.Close(); } catch (IOException) { }
            if (!ff.WaitForExit(timeoutMs)) { try { ff.Kill(); } catch (Exception) { } return false; }
            ff.WaitForExit();
            return ff.ExitCode == 0;
        }

        public void Dispose() {
            if (ff != null && !ff.HasExited) { try { ff.Kill(); } catch (Exception) { } }
            sha.Dispose();
        }
    }
}
