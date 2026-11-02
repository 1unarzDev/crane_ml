// Ghost theater HUD, drawn into the captured frame after the camera (Crane.Theater.HudRenderer). Vertices are in
// pixels (origin bottom-left); _Screen = (width, height, flipY). Pass 0: solid shapes; pass 1: text from a font atlas.
Shader "Hidden/Crane/TheaterHud" {
    Properties { _MainTex ("Font", 2D) = "white" {} }
    SubShader {
        Tags { "RenderType" = "Transparent" "Queue" = "Overlay" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off ZTest Always Cull Off
        HLSLINCLUDE
        #pragma target 4.5
        float4 _Screen;
        float4 Px(float2 p) { return float4(p.x / _Screen.x * 2 - 1, (p.y / _Screen.y * 2 - 1) * _Screen.z, 0.5, 1); }
        ENDHLSL
        Pass {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            struct A { float3 v : POSITION; float4 c : COLOR; };
            struct V { float4 pos : SV_POSITION; float4 c : COLOR; };
            V vert(A a) { V o; o.pos = Px(a.v.xy); o.c = a.c; return o; }
            float4 frag(V i) : SV_Target { return i.c; }
            ENDHLSL
        }
        Pass {
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
