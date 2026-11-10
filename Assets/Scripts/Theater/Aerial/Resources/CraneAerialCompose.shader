// Aerial demo compositor: pass 0 = onboard sensor post (optional noise, rolling-shutter shear, mono), pass 1 = textured
// rectangle blit (split-screen / picture-in-picture, crop window, alpha). Rects are in destination UV (origin bottom-left).
Shader "Hidden/Crane/AerialCompose" {
    Properties { _MainTex ("Source", 2D) = "black" {} }
    SubShader {
        Tags { "RenderType" = "Opaque" }
        ZWrite Off ZTest Always Cull Off
        HLSLINCLUDE
        #pragma target 4.5
        Texture2D _MainTex; SamplerState sampler_linear_clamp;
        float4 _Rect;      // x y w h in destination uv
        float4 _UvWin;     // u0 v0 u1 v1 of the source
        float4 _Params;    // x flipY (-1 on top-origin APIs), y alpha, z noise sigma, w seed
        float4 _Sensor;    // x shear (uv width per frame height, top row vs bottom row), y mono 0/1, z width px, w height px
        struct V { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; float2 duv : TEXCOORD1; };
        V vert(uint id : SV_VertexID) {
            const float2 c[6] = { float2(0,0), float2(1,0), float2(1,1), float2(0,0), float2(1,1), float2(0,1) };
            float2 q = c[id];
            V o;
            float2 d = _Rect.xy + q * _Rect.zw;
            o.pos = float4(d.x * 2 - 1, (d.y * 2 - 1) * _Params.x, 0.5, 1);
            o.uv = lerp(_UvWin.xy, _UvWin.zw, q);
            o.duv = d;
            return o;
        }
        float h21(float2 p, float s) { float3 p3 = frac(float3(p.xyx) * 0.1031 + s * 0.0137); p3 += dot(p3, p3.yzx + 33.33); return frac((p3.x + p3.y) * p3.z); }
        ENDHLSL
        Pass {
            Blend Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 frag(V i) : SV_Target {
                float2 uv = i.uv;
                uv.x += _Sensor.x * (1.0 - uv.y);                       // rolling shutter: later rows read later
                float4 c = _MainTex.SampleLevel(sampler_linear_clamp, uv, 0);
                if (_Sensor.y > 0.5) c.rgb = dot(c.rgb, float3(0.299, 0.587, 0.114)).xxx;
                if (_Params.z > 0) {
                    float2 px = floor(i.duv * _Sensor.zw);
                    float n = (h21(px, _Params.w) + h21(px + 17.0, _Params.w + 3.0) + h21(px + 41.0, _Params.w + 7.0) - 1.5) * 2.0;   // ~N(0,1)
                    float l = dot(c.rgb, float3(0.299, 0.587, 0.114));
                    c.rgb += n * _Params.z * (0.35 + 0.65 * sqrt(saturate(l)));   // signal-dependent sigma
                }
                return float4(saturate(c.rgb), 1);
            }
            ENDHLSL
        }
        Pass {
            Blend SrcAlpha OneMinusSrcAlpha
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 frag(V i) : SV_Target {
                float4 c = _MainTex.SampleLevel(sampler_linear_clamp, i.uv, 0);
                return float4(c.rgb, _Params.y);
            }
            ENDHLSL
        }
    }
}
