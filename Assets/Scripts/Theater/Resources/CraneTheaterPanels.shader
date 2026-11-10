// The theater look's data panels (Crane.Theater.TheaterPanels), drawn into the captured frame after the camera.
// Vertices are in pixels (origin bottom-left); _Screen = (width, height, flipY). Pass 0: signed-distance rounded
// boxes (panels, bars, lines, dots) with a border, a soft shadow and frosted glass from _Blur (a blurred copy of the
// frame); pass 1: text from a font atlas.
Shader "Hidden/Crane/TheaterPanels" {
    Properties { _MainTex ("Font", 2D) = "white" {} _Blur ("Blur", 2D) = "black" {} }
    SubShader {
        Tags { "RenderType" = "Transparent" "Queue" = "Overlay" }
        ZWrite Off ZTest Always Cull Off
        HLSLINCLUDE
        #pragma target 4.5
        float4 _Screen;
        float4 Px(float2 p) { return float4(p.x / _Screen.x * 2 - 1, (p.y / _Screen.y * 2 - 1) * _Screen.z, 0.5, 1); }
        ENDHLSL
        Pass {
            Blend One OneMinusSrcAlpha
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            Texture2D _Blur; SamplerState sampler_linear_clamp;
            struct A { float3 v : POSITION; float4 c : COLOR; float4 l : TEXCOORD0; float4 s : TEXCOORD1; float4 b : TEXCOORD2; float4 g : TEXCOORD3; };
            struct V { float4 pos : SV_POSITION; float4 c : COLOR; float4 l : TEXCOORD0; float4 s : TEXCOORD1; float4 b : TEXCOORD2; float4 g : TEXCOORD3; float2 sp : TEXCOORD4; };
            V vert(A a) { V o; o.pos = Px(a.v.xy); o.c = a.c; o.l = a.l; o.s = a.s; o.b = a.b; o.g = a.g; o.sp = a.v.xy / _Screen.xy; return o; }
            float Box(float2 p, float2 h, float r) { float2 q = abs(p) - h + r; return length(max(q, 0)) + min(max(q.x, q.y), 0) - r; }
            float4 frag(V i) : SV_Target {
                float2 p = i.l.xy, h = i.l.zw;
                float r = min(i.s.x, min(h.x, h.y));
                float d = Box(p, h, r);
                float aa = max(fwidth(d), 0.6);
                float body = saturate(0.5 - d / aa);
                float inner = saturate(0.5 - (d + i.s.y) / aa);
                float edge = saturate(body - inner);
                float4 fill = i.c;
                // a faint vertical sheen: lighter toward the top edge
                fill.rgb *= 1 + i.g.y * saturate(p.y / max(h.y, 1)) ;
                float4 o = 0;
                if (i.g.x > 0) {
                    float2 uv = i.sp; uv.y = _Screen.z < 0 ? 1 - uv.y : uv.y;
                    float2 t = 1.0 / float2(240, 135);
                    float3 blur = (_Blur.Sample(sampler_linear_clamp, uv + t * float2(0.7, 0.4)).rgb + _Blur.Sample(sampler_linear_clamp, uv - t * float2(0.7, 0.4)).rgb
                                 + _Blur.Sample(sampler_linear_clamp, uv + t * float2(-0.4, 0.7)).rgb + _Blur.Sample(sampler_linear_clamp, uv + t * float2(0.4, -0.7)).rgb) * 0.25;
                    float glass = i.g.x * inner;
                    o.rgb += blur * 0.55 * glass * (1 - fill.a);   // frosted and darkened
                    o.a += glass * (1 - fill.a);
                }
                o.rgb += fill.rgb * fill.a * inner;
                o.a += fill.a * inner;
                o.rgb = o.rgb * (1 - i.b.a * edge) + i.b.rgb * i.b.a * edge;
                o.a = o.a * (1 - i.b.a * edge) + i.b.a * edge;
                if (i.s.z > 0) {
                    float sh = (1 - smoothstep(-i.s.z * 0.25, i.s.z, d + 2)) * i.s.w * (1 - body);
                    o.a += sh * (1 - o.a);
                }
                return o;
            }
            ENDHLSL
        }
        Pass {
            Blend SrcAlpha OneMinusSrcAlpha
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            Texture2D _MainTex; SamplerState sampler_MainTex;
            struct A { float3 v : POSITION; float4 c : COLOR; float2 uv : TEXCOORD0; };
            struct V { float4 pos : SV_POSITION; float4 c : COLOR; float2 uv : TEXCOORD0; };
            V vert(A a) { V o; o.pos = Px(a.v.xy); o.c = a.c; o.uv = a.uv; return o; }
            float4 frag(V i) : SV_Target { return float4(i.c.rgb, i.c.a * _MainTex.Sample(sampler_MainTex, i.uv).a); }
            ENDHLSL
        }
    }
}
