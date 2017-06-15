Shader "Custom/Negative" {
	Properties {
		_Invert("Invert Percentage", Range(0,1)) = 1.0
		_Color ("Color", Color) = (1,1,1,1)
		_MainTex ("Albedo (RGB)", 2D) = "white" {}
		_Glossiness ("Smoothness", Range(0,1)) = 0.5
		_Metallic ("Metallic", Range(0,1)) = 0.0
	}
	SubShader {
		Tags { "RenderType"="Opaque" }
		LOD 200
		
		CGPROGRAM
		// Physically based Standard lighting model, and enable shadows on all light types
		#pragma surface surf Standard fullforwardshadows

		// Use shader model 3.0 target, to get nicer looking lighting
		#pragma target 3.0

		sampler2D _MainTex;

		struct Input {
			float2 uv_MainTex;
		};

		half _Invert;
		half _Glossiness;
		half _Metallic;
		fixed4 _Color;

		void surf (Input IN, inout SurfaceOutputStandard o) {
			// Albedo comes from a texture tinted by color
			fixed4 c = tex2D (_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = abs(c.rgb - _Invert);
			// Metallic and smoothness come from slider variables
			o.Metallic = _Metallic;//  abs(_Metallic - _Invert);
			o.Smoothness = _Glossiness;//  abs(_Glossiness - _Invert);
			o.Alpha = c.a;
		}
		ENDCG
	}
	FallBack "Diffuse"
}
