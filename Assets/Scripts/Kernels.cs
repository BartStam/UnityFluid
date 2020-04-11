using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Kernel {
	private readonly float smoothingRadius;
	private readonly float smoothingRadiusSquared;

	// Precalculated kernel constants for efficiency
	private readonly float poly6;
	private readonly float spikyGradient;
	private readonly float viscosityLaplacian;

	public Kernel(float smoothingRadius) {
		this.smoothingRadius = smoothingRadius;
		smoothingRadiusSquared = smoothingRadius * smoothingRadius;

		poly6 = 315.0f / (64.0f * Mathf.PI * Mathf.Pow(smoothingRadius, 9.0f));
		spikyGradient = -45.0f / (Mathf.PI * Mathf.Pow(smoothingRadius, 6.0f));
		viscosityLaplacian = 45.0f / (Mathf.PI * Mathf.Pow(smoothingRadius, 6.0f));
	}

	// Equation (20) in Muller et al. (2003)
	public float Poly6(Vector3 r) {
		float distance = r.magnitude;

		if (distance > smoothingRadius) { return 0.0f; }
		
		return poly6 * Mathf.Pow((smoothingRadiusSquared - Mathf.Pow(distance, 2.0f)), 3.0f);
	}

	// Desbrun's "Spiky" kernel
	// Equation (21) in Muller et al. (2003)
	public Vector3 SpikyGradient(Vector3 r) {
		float distance = r.magnitude;

		if (distance > smoothingRadius) { return Vector3.zero; }

		return spikyGradient * r.normalized * Mathf.Pow(smoothingRadius - distance, 2.0f);
	}

	// Equation (22) in Muller et al. (2003)
	public float ViscosityLaplacian(Vector3 r) {
		float distance = r.magnitude;

		if (distance > smoothingRadius) { return 0.0f; }

		return viscosityLaplacian * (smoothingRadius - distance);
	}
}
