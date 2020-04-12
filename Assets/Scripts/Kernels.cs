using Unity.Mathematics;
using UnityEngine;

public class Kernels {
	// Equation (20) in Muller et al. (2003)
	public static float Poly6(float3 r, float h) {
		float distance = math.length(r);

		if (distance > h) { return 0.0f; }
		
		return (315.0f / (64.0f * Mathf.PI * Mathf.Pow(h, 9.0f))) * Mathf.Pow((Mathf.Pow(h, 2.0f) - Mathf.Pow(distance, 2.0f)), 3.0f);
	}

	// Desbrun's "Spiky" kernel
	// Equation (21) in Muller et al. (2003)
	public static float3 SpikyGradient(float3 r, float h) {
		float distance = math.length(r);

		if (distance > h) { return float3.zero; }

		return (-45.0f / (math.PI * math.pow(h, 6.0f))) * math.normalizesafe(r) * math.pow(h - distance, 2.0f);
	}

	// Equation (22) in Muller et al. (2003)
	public static float ViscosityLaplacian(float3 r, float h) {
		float distance = math.length(r);

		if (distance > h) { return 0.0f; }

		return (45.0f / (Mathf.PI * Mathf.Pow(h, 6.0f))) * (h - distance);
	}
}
