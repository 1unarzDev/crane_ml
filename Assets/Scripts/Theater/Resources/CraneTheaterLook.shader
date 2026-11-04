// The theater look's own layers (Crane.Theater.TheaterLook). Passes 0-2 run in an HDRP custom pass before
// post-processing, so they are exposed, fogged, tonemapped and graded with the scene; colours are given in the
// sky's units and multiplied by the current exposure. Pass 3 runs after the overlays (film grain). Nothing reads a
// clock: noise is seeded by world position or by _Frame.
Shader "Hidden/Crane/TheaterLook" {
    SubShader {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent" }
        HLSLINCLUDE
        #pragma target 4.5
        float4x4 _TheaterVP;
        float3 _CamPos;
        float _Frame;
        Texture2D _ExposureTexture;
        float Exposure() { return _ExposureTexture.Load(int3(0, 0, 0)).x; }
        float4 Clip(float3 w) { return mul(_TheaterVP, float4(w, 1)); }
        float Hash(float2 p) { p = frac(p * float2(443.897, 441.423)); p += dot(p, p.yx + 19.19); return frac((p.x + p.y) * p.x); }
        float Noise(float2 p) {
            float2 i = floor(p), f = frac(p);
            float2 u = f * f * (3 - 2 * f);
            return lerp(lerp(Hash(i), Hash(i + float2(1, 0)), u.x), lerp(Hash(i + float2(0, 1)), Hash(i + float2(1, 1)), u.x), u.y);
        }
        float Fbm(float2 p) { return 0.55 * Noise(p) + 0.3 * Noise(p * 2.13 + 7.1) + 0.15 * Noise(p * 4.37 + 3.3); }
        ENDHLSL

        // 0: horizon hills (opaque silhouettes, atmospheric haze toward their base and with distance)
        Pass {
            Name "Hills"
            ZWrite On ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 _Haze, _Hill;
            struct A { float3 v : POSITION; float4 uv : TEXCOORD0; };
            struct V { float4 pos : SV_POSITION; float4 uv : TEXCOORD0; float3 w : TEXCOORD1; };
            V vert(A a) { V o; o.w = a.v; o.pos = Clip(a.v); o.uv = a.uv; return o; }
            float4 frag(V i) : SV_Target {
                float far = i.uv.y;                        // 0 near range, 1 far range
                float up = i.uv.x;                         // 0 base, 1 ridge
                float mist = 1 - smoothstep(0.0, 0.45, up);
                float tex = Fbm(float2(i.uv.z * 90, i.w.y * 0.08)) - 0.5;
                float haze = saturate(lerp(0.42, 0.72, far) + mist * 0.3 + tex * 0.06);
                float3 c = lerp(_Hill.rgb, _Haze.rgb, haze);
                return float4(c * Exposure(), 1);
            }
            ENDHLSL
        }

        // 1: hero wakes on the water: two foam arms that spread with age and a churned core behind the hull
        Pass {
            Name "Wake"
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 _Foam;
            struct A { float3 v : POSITION; float4 uv : TEXCOORD0; };
            struct V { float4 pos : SV_POSITION; float4 uv : TEXCOORD0; float3 w : TEXCOORD1; };
            V vert(A a) { V o; o.w = a.v; o.pos = Clip(a.v); o.uv = a.uv; return o; }
            float4 frag(V i) : SV_Target {
                float age = i.uv.x, x = abs(i.uv.y), fade = i.uv.z;
                float n = Fbm(i.w.xz * 2.6);
                float n2 = Fbm(i.w.xz * 7.3 + 11.0);
                float arms = smoothstep(0.55, 0.9, x) * (1 - smoothstep(0.9, 1.0, x));
                float core = (1 - smoothstep(0.0, 0.45, x)) * saturate(1 - age * 2.2);
                float lace = smoothstep(0.45, 0.8, n * 0.7 + n2 * 0.5 - age * 0.25);
                float a = (arms * (0.75 + 0.25 * n2) * lace + core * (0.45 + 0.55 * n)) * pow(saturate(1 - age), 1.6) * fade;
                return float4(_Foam.rgb * Exposure(), saturate(a * 0.85));
            }
            ENDHLSL
        }

        // 2: ghost ink: each ghost darkens the water under its body (multiplied), the overlays add its rim later
        Pass {
            Name "Ink"
            Blend DstColor Zero
            ZWrite Off ZTest LEqual Cull Back
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            struct Ghost { float3 pos; float yaw; float pitch; float roll; float scale; float pad; float4 color; };
            StructuredBuffer<Ghost> _Ghosts;
            float _Ink;
            float3x3 RotY(float a) { float s = sin(a), c = cos(a); return float3x3(c, 0, s, 0, 1, 0, -s, 0, c); }
            float3x3 RotX(float a) { float s = sin(a), c = cos(a); return float3x3(1, 0, 0, 0, c, -s, 0, s, c); }
            float3x3 RotZ(float a) { float s = sin(a), c = cos(a); return float3x3(c, -s, 0, s, c, 0, 0, 0, 1); }
            struct A { float3 v : POSITION; float3 n : NORMAL; uint id : SV_InstanceID; };
            struct V { float4 pos : SV_POSITION; float3 n : TEXCOORD0; float3 w : TEXCOORD1; float4 c : COLOR; };
            V vert(A a) {
                Ghost g = _Ghosts[a.id];
                float3x3 R = mul(RotY(radians(g.yaw)), mul(RotX(radians(g.pitch)), RotZ(radians(g.roll))));
                V o;
                o.w = g.pos + mul(R, a.v * g.scale);
                o.n = mul(R, a.n);
                o.pos = Clip(o.w);
                o.c = g.color;
                return o;
            }
            float4 frag(V i) : SV_Target {
                float3 n = normalize(i.n), v = normalize(_CamPos - i.w);
                float facing = saturate(abs(dot(n, v)));
                float k = _Ink * saturate(i.c.a * 1.4) * (0.35 + 0.65 * facing);
                float3 tint = lerp(float3(1, 1, 1), saturate(i.c.rgb * 0.35 + float3(0.05, 0.07, 0.1)), k);
                return float4(tint, 1);
            }
            ENDHLSL
        }

        // 3: film grain (multiplied, seeded by the frame index and the pixel; mid-tone weighted by its shape)
        Pass {
            Name "Grain"
            Blend DstColor Zero
            ZWrite Off ZTest Always Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float _Grain;
            struct V { float4 pos : SV_POSITION; };
            V vert(uint id : SV_VertexID) {
                float2 uv = float2((id << 1) & 2, id & 2);
                V o; o.pos = float4(uv * 2 - 1, 0.5, 1); return o;
            }
            float4 frag(V i) : SV_Target {
                float2 p = floor(i.pos.xy);
                float f = fmod(_Frame, 997.0);
                float a = Hash(p * 0.731 + f * 13.17), b = Hash(p.yx * 1.137 + f * 7.31 + 3.1);
                float g = (a + b - 1.0);                   // triangular, zero mean
                return float4((1 + g * _Grain).xxx, 1);
            }
            ENDHLSL
        }
    }
}
