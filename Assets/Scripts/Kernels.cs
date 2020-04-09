using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Kernels {
	// Equation (20) in Muller et al. (2003)
	public static float Poly6(Vector3 r, float h) {
		float distance = r.magnitude;

		if (distance > h) { return 0.0f; }
		
		return (315.0f / (64.0f * Mathf.PI * Mathf.Pow(h, 9.0f))) * Mathf.Pow((Mathf.Pow(h, 2.0f) - Mathf.Pow(distance, 2.0f)), 3.0f);
	}

	// Desbrun's "Spiky" kernel
	// Equation (21) in Muller et al. (2003)
	public static Vector3 SpikyGradient(Vector3 r, float h) {
		float distance = r.magnitude;

		if (distance > h) {
			return Vector3.zero;
		}

		return -1.0f * (45.0f / (Mathf.PI * Mathf.Pow(h, 6.0f))) * r.normalized * Mathf.Pow(h - distance, 2.0f);
	}

	// Equation (22) in Muller et al. (2003)
	public static float ViscosityLaplacian(Vector3 r, float h) {
		float distance = r.magnitude;

		if (distance > h) { return 0.0f; }

		return (45.0f / (Mathf.PI * Mathf.Pow(h, 6.0f))) * (h - distance);
	}
}
