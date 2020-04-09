﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour {
	public GameObject particlePrefab;

	// Options
	public Vector3Int cubeDimensions;
	public float particleSpacing;
	public float smoothingRadius;

	// Physical constants
	public float restDensity;
	public float viscosity;
	public float gasConstant;
	private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

	private Particle[] particles;

	void Start() {
		InstantiateParticles();
	}

    void FixedUpdate() {
		UpdateForces();

		for (int i = 0; i < particles.Length; i++) {
			particles[i].velocity += Time.fixedDeltaTime * particles[i].CombinedForce / particles[i].density;
			particles[i].Position += Time.fixedDeltaTime * particles[i].velocity;
		}
	}

	private void InstantiateParticles() {
		particles = new Particle[cubeDimensions.x * cubeDimensions.y * cubeDimensions.z];

		for (int x = 0; x < cubeDimensions.x; x++) {
			for (int y = 0; y < cubeDimensions.y; y++) {
				for (int z = 0; z < cubeDimensions.z; z++) {
					GameObject p = Instantiate(particlePrefab, new Vector3(x * particleSpacing, y * particleSpacing, z * particleSpacing), Quaternion.identity);

					particles[x * cubeDimensions.y * cubeDimensions.z + y * cubeDimensions.z + z] = new Particle(p);
				}
			}
		}
	}

	private void UpdateForces() {
		// First recalculate particle density and pressure before we calculate forces
		for (int i = 0; i < particles.Length; i++) {
			particles[i].density = 0.0f;

			for (int j = 0; j < particles.Length; j++) {
				// Evaluate density at particle location
				// Equations (3) and (20) in Muller et al. (2003)
				particles[i].density += Kernels.Poly6(particles[i].Position - particles[j].Position, smoothingRadius);
			}

			// Evaluate pressure at particle location using Desbrun's equation
			// Equation (12) in Muller et al. (2003)
			particles[i].pressure = gasConstant * (particles[i].density - restDensity);

			// Evaluate gravity at particle location
			// The force is scaled by the density instead of the mass
			particles[i].gravityForce = gravity * particles[i].density;
		}

		for (int i = 0; i < particles.Length; i++) {
			particles[i].pressureForce = Vector3.zero;
			particles[i].viscosityForce = Vector3.zero;

			for (int j = 0; j < particles.Length; j++) {
				if (i == j) { continue; }
				// Calculate pressure forces
				// Equations (10) and (21) in Muller et al. (2003)
				particles[i].pressureForce -= ((particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density)) * Kernels.SpikyGradient(particles[i].Position - particles[j].Position, smoothingRadius);

				// Calculate the viscosity force
				// Equations (14) and (22) in Muller et al. (2003)
				particles[i].viscosityForce += viscosity * ((particles[j].velocity - particles[i].velocity) / particles[j].density) * Kernels.ViscosityLaplacian(particles[i].Position - particles[j].Position, smoothingRadius);
			}
		}
	}
}