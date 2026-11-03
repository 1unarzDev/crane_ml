// Ghost theater world-space layers, drawn by Crane.Theater.TheaterPass (an HDRP custom pass after post-processing,
// against the camera depth). Every pass is additive and writes no depth, so thousands of overlapping ghosts need
// no sorting and density reads as brightness. Positions are world space; _TheaterVP is the camera's GPU
// view-projection (set per draw), so nothing depends on HDRP's camera-relative matrices.
Shader "Hidden/Crane/TheaterWorld" {
    Properties { _Heat ("Heat", 2D) = "black" {} }
    SubShader {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent" }
        HLSLINCLUDE
        #pragma target 4.5
        float4x4 _TheaterVP;
        float3 _CamPos;
        float3x3 RotY(float a) { float s = sin(a), c = cos(a); return float3x3(c, 0, s, 0, 1, 0, -s, 0, c); }
        float3x3 RotX(float a) { float s = sin(a), c = cos(a); return float3x3(1, 0, 0, 0, c, -s, 0, s, c); }
        float3x3 RotZ(float a) { float s = sin(a), c = cos(a); return float3x3(c, -s, 0, s, c, 0, 0, 0, 1); }
        float4 Clip(float3 w) { return mul(_TheaterVP, float4(w, 1)); }
        ENDHLSL

        // 0: ghosts, one instance per element of _Ghosts (fresnel shell: bright rim, faint core)
        Pass {
            Name "Ghost"
            Blend One One
            ZWrite Off ZTest LEqual Cull Back
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            struct Ghost { float3 pos; float yaw; float pitch; float roll; float scale; float pad; float4 color; };
            StructuredBuffer<Ghost> _Ghosts;
            float _Core, _Rim;
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
                float rim = pow(1 - saturate(abs(dot(n, v))), 2.2);
                float top = saturate(n.y) * 0.35;
                return float4(i.c.rgb * i.c.a * (_Core + top + _Rim * rim), 0);
            }
            ENDHLSL
        }

        // 1: ribbons (trails and comet tails): uv.x = run time of the vertex, uv.y = -1..1 across
        Pass {
            Name "Ribbon"
            Blend SrcAlpha One
            ZWrite Off ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float _Now, _Tail, _Floor, _Gain;
            struct A { float3 v : POSITION; float4 c : COLOR; float2 uv : TEXCOORD0; };
            struct V { float4 pos : SV_POSITION; float4 c : COLOR; float2 uv : TEXCOORD0; };
            V vert(A a) { V o; o.pos = Clip(a.v); o.c = a.c; o.uv = a.uv; return o; }
            float4 frag(V i) : SV_Target {
                float age = _Now - i.uv.x;
                clip(age);
                float fade = _Tail > 0 ? saturate(1 - age / _Tail) : 1;
                fade = max(fade * fade, _Floor);
                float edge = 1 - smoothstep(0.55, 1.0, abs(i.uv.y));
                return float4(i.c.rgb, i.c.a * fade * edge * _Gain);
            }
            ENDHLSL
        }

        // 2: event markers: expanding rings on the surface, one quad per element of _Markers
        Pass {
            Name "Marker"
            Blend SrcAlpha One
            ZWrite Off ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            struct Marker { float3 pos; float radius; float4 color; float age; float3 pad; };
            StructuredBuffer<Marker> _Markers;
            struct V { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; float4 c : COLOR; float age : TEXCOORD1; };
            V vert(uint vid : SV_VertexID, uint id : SV_InstanceID) {
                Marker m = _Markers[id];
                float2 q[6] = { float2(-1, -1), float2(1, -1), float2(1, 1), float2(-1, -1), float2(1, 1), float2(-1, 1) };
                float2 uv = q[vid];
                float r = m.radius * (0.6 + 1.4 * saturate(m.age));
                V o;
                o.pos = Clip(m.pos + float3(uv.x * r, 0.04, uv.y * r));
                o.uv = uv; o.c = m.color; o.age = m.age;
                return o;
            }
            float4 frag(V i) : SV_Target {
                float d = length(i.uv);
                float ring = smoothstep(0.78, 0.9, d) * (1 - smoothstep(0.92, 1.0, d));
                float core = (1 - smoothstep(0.0, 0.35, d)) * 0.35 * saturate(1 - i.age * 2);
                float a = (ring + core) * saturate(1 - i.age) * i.c.a;
                return float4(i.c.rgb, a);
            }
            ENDHLSL
        }

        // 3: path density (r) and failure heat (g) on the surface over _HeatRect (world x0, z0, x1, z1)
        Pass {
            Name "Heat"
            Blend SrcAlpha One
            ZWrite Off ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            Texture2D _Heat; SamplerState sampler_linear_clamp;
            float4 _HeatRect; float _HeatY, _DensityGain, _FailGain;
            float4 _DensityColor, _FailColor;
            struct V { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };
            V vert(uint vid : SV_VertexID) {
                float2 q[6] = { float2(0, 0), float2(1, 0), float2(1, 1), float2(0, 0), float2(1, 1), float2(0, 1) };
                float2 uv = q[vid];
                V o;
                o.pos = Clip(float3(lerp(_HeatRect.x, _HeatRect.z, uv.x), _HeatY, lerp(_HeatRect.y, _HeatRect.w, uv.y)));
                o.uv = uv;
                return o;
            }
            float4 frag(V i) : SV_Target {
                float4 h = _Heat.Sample(sampler_linear_clamp, i.uv);
                float d = saturate(h.r * _DensityGain), f = saturate(h.g * _FailGain);
                float3 c = _DensityColor.rgb * d + _FailColor.rgb * f;
                return float4(c, saturate(max(d * _DensityColor.a, f * _FailColor.a)));
            }
            ENDHLSL
        }

        // 4: wash, the one alpha-blended layer: a tinted plane just above the water, under everything else the pass
        // draws, that darkens the water (and its foam) towards a study's figure background, with a screen vignette.
        // Objects that stand out of the water (buoys, hero boats) pass the depth test and stay lit.
        Pass {
            Name "Wash"
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off ZTest LEqual Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 _WashRect; float _WashY; float4 _WashColor; float4 _WashView;   // view: w, h, vignette, vignette start
            struct V { float4 pos : SV_POSITION; };
            V vert(uint vid : SV_VertexID) {
                float2 q[6] = { float2(0, 0), float2(1, 0), float2(1, 1), float2(0, 0), float2(1, 1), float2(0, 1) };
                float2 uv = q[vid];
                V o;
                o.pos = Clip(float3(lerp(_WashRect.x, _WashRect.z, uv.x), _WashY, lerp(_WashRect.y, _WashRect.w, uv.y)));
                return o;
            }
            float4 frag(V i) : SV_Target {
                float2 p = i.pos.xy / max(_WashView.xy, 1) * 2 - 1;
                p.x *= _WashView.x / max(_WashView.y, 1) * 0.75;
                float vig = smoothstep(_WashView.w, 1.6, length(p)) * _WashView.z;
                return float4(_WashColor.rgb * (1 - 0.5 * vig), saturate(_WashColor.a + vig * (1 - _WashColor.a)));
            }
            ENDHLSL
        }

        // 5: far wash: the same plane where something stands in front of it (ZTest Greater) far from the camera
        // (_WashFar: distances where it starts and is full) or in the frame's corners (_WashFar.z: vignette level), so a
        // distant shore or a corner of foam reads as the same navy; buoys and boats near the course keep their colour.
        Pass {
            Name "WashFar"
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off ZTest Greater Cull Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 _WashRect; float _WashY; float4 _WashColor; float4 _WashView; float4 _WashFar;
            struct V { float4 pos : SV_POSITION; float3 w : TEXCOORD0; };
            V vert(uint vid : SV_VertexID) {
                float2 q[6] = { float2(0, 0), float2(1, 0), float2(1, 1), float2(0, 0), float2(1, 1), float2(0, 1) };
                float2 uv = q[vid];
                V o;
                o.w = float3(lerp(_WashRect.x, _WashRect.z, uv.x), _WashY, lerp(_WashRect.y, _WashRect.w, uv.y));
                o.pos = Clip(o.w);
                return o;
            }
            float4 frag(V i) : SV_Target {
                float2 p = i.pos.xy / max(_WashView.xy, 1) * 2 - 1;
                p.x *= _WashView.x / max(_WashView.y, 1) * 0.75;
                float vig = smoothstep(_WashView.w, 1.6, length(p)) * _WashView.z;
                float far = smoothstep(_WashFar.x, _WashFar.y, distance(i.w, _CamPos));
                float corner = _WashFar.z > 0 ? smoothstep(_WashFar.z, _WashFar.z + 0.15, vig) : 0;
                float a = max(far, corner) * saturate(_WashColor.a + vig * (1 - _WashColor.a));
                clip(a - 0.002);
                return float4(_WashColor.rgb * (1 - 0.5 * vig), a);
            }
            ENDHLSL
        }
    }
}
