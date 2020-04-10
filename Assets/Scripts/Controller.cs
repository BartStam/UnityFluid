using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour {
	public GameObject particlePrefab;
	private float particleRadius;

	// Options
	public Vector3Int cubeDimensions;
	public float particleSpacing;
	public float smoothingRadius;
	public Vector3 minBound;
	public Vector3 maxBound;

	// Physical constants
	public float restDensity;
	public float viscosity;
	public float gasConstant;
	public Vector3 external = new Vector3(0.0f, -9.81f, 0.0f);

	public Particle[] particles;

	void Start() {
		particleRadius = particlePrefab.transform.localScale.x / 2.0f;
		InstantiateParticles();
	}

    void FixedUpdate() {
		UpdateForces();

		for (int i = 0; i < particles.Length; i++) {
			particles[i].velocity += Time.fixedDeltaTime * particles[i].CombinedForce / particles[i].density;
			particles[i].Position += Time.fixedDeltaTime * particles[i].velocity;
			HandleCollision(particles[i]);
			particles[i].velocity = 0.99f * particles[i].velocity;
		}
	}

	private void InstantiateParticles() {
		particles = new Particle[cubeDimensions.x * cubeDimensions.y * cubeDimensions.z];

		for (int x = 0; x < cubeDimensions.x; x++) {
			for (int y = 0; y < cubeDimensions.y; y++) {
				for (int z = 0; z < cubeDimensions.z; z++) {
					GameObject p = Instantiate(particlePrefab, new Vector3(
						x * particleSpacing + Random.Range(-0.2f, 0.2f),
						4.0f + y * particleSpacing + Random.Range(-0.2f, 0.2f),
						z * particleSpacing + Random.Range(-0.2f, 0.2f)),
						Quaternion.identity);

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
			particles[i].pressure = Mathf.Max(0.0f, gasConstant * (particles[i].density - restDensity));

			// Evaluate gravity at particle location
			// The force is scaled by the density instead of the mass
			particles[i].externalForce = external * particles[i].density;
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

	private void HandleCollision(Particle p) {
		// If we are colliding with the floor
		if (p.Position.y < minBound.y + particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.y = minBound.y + particleRadius;
			p.Position = newPosition;
			p.velocity.y = 0.8f * -p.velocity.y;
		}

		if (p.Position.y > maxBound.y - particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.y = maxBound.y - particleRadius;
			p.Position = newPosition;
			p.velocity.y = 0.8f * -p.velocity.y;
		}

		if (p.Position.x < minBound.x + particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.x = minBound.x + particleRadius;
			p.Position = newPosition;
			p.velocity.x = 0.8f * -p.velocity.x;
		}

		if (p.Position.x > maxBound.x - particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.x = maxBound.x - particleRadius;
			p.Position = newPosition;
			p.velocity.x = 0.8f * -p.velocity.x;
		}

		if (p.Position.z < minBound.z + particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.z = minBound.z + particleRadius;
			p.Position = newPosition;
			p.velocity.z = 0.8f * -p.velocity.z;
		}

		if (p.Position.z > maxBound.z - particleRadius) {
			Vector3 newPosition = p.Position;
			newPosition.z = maxBound.z - particleRadius;
			p.Position = newPosition;
			p.velocity.z = 0.8f * -p.velocity.z;
		}
	}
}
