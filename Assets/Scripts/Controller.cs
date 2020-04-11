using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour {
	public GameObject particlePrefab;
	private float particleRadius;

	// Options
	public float timestep;
	public Vector3Int cubeDimensions;
	public float particleSpacing;
	public float smoothingRadius;
	public Vector3 minBound;
	public Vector3 maxBound;
	public float boundDamping = -0.5f;
	private float gravityMultiplier;

	// Physical constants
	public float restDensity;
	public float viscosity;
	public float gasConstant;
	public float particleMass;
	public Vector3 external;
	private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

	public Particle[] particles;

	void Start() {
		particleRadius = particlePrefab.transform.localScale.x / 2.0f;
		InstantiateParticles();
		gravityMultiplier = 10 * (1.0f / timestep);
	}

    void FixedUpdate() {
		UpdateDensityPressure();
		UpdateForces();
		Integrate();
		HandleCollisions();
		CommitProposedPosition();
	}

	private void InstantiateParticles() {
		particles = new Particle[cubeDimensions.x * cubeDimensions.y * cubeDimensions.z];

		for (int x = 0; x < cubeDimensions.x; x++) {
			for (int y = 0; y < cubeDimensions.y; y++) {
				for (int z = 0; z < cubeDimensions.z; z++) {
					GameObject p = Instantiate(particlePrefab, new Vector3(
						x * particleSpacing + Random.Range(-0.1f, 0.1f),
						4.0f + y * particleSpacing + Random.Range(-0.1f, 0.1f),
						z * particleSpacing + Random.Range(-0.1f, 0.1f)),
						Quaternion.identity);

					particles[x * cubeDimensions.y * cubeDimensions.z + y * cubeDimensions.z + z] = new Particle(p);
				}
			}
		}
	}

	private void UpdateDensityPressure() {
		// First recalculate particle density and pressure before we calculate forces
		for (int i = 0; i < particles.Length; i++) {
			particles[i].density = 0.0f;

			for (int j = 0; j < particles.Length; j++) {
				// Evaluate density at particle location
				// Equations (3) and (20) in Muller et al. (2003)
				particles[i].density += particleMass * Kernels.Poly6(particles[i].Position - particles[j].Position, smoothingRadius);
			}

			// Evaluate pressure at particle location using Desbrun's equation
			// Equation (12) in Muller et al. (2003)
			particles[i].pressure = Mathf.Max(0.0f, gasConstant * (particles[i].density - restDensity));
		}
	}

	private void UpdateForces() {
		for (int i = 0; i < particles.Length; i++) {
			particles[i].pressureForce = Vector3.zero;
			particles[i].viscosityForce = Vector3.zero;

			for (int j = 0; j < particles.Length; j++) {
				if (i == j) { continue; }

				// Calculate pressure forces
				// Equations (10) and (21) in Muller et al. (2003)
				particles[i].pressureForce -= particleMass * ((particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density)) * Kernels.SpikyGradient(particles[i].Position - particles[j].Position, smoothingRadius);

				// Calculate the viscosity force
				// Equations (14) and (22) in Muller et al. (2003)
				particles[i].viscosityForce += particleMass * viscosity * ((particles[j].velocity - particles[i].velocity) / particles[j].density) * Kernels.ViscosityLaplacian(particles[i].Position - particles[j].Position, smoothingRadius);
			}

			// Evaluate gravity and external force at particle location
			// The force is scaled by the density instead of the mass
			particles[i].externalForce = gravityMultiplier * (external + gravity) * particles[i].density;
		}
	}

	private void Integrate() {
		for (int i = 0; i < particles.Length; i++) {
			particles[i].velocity += timestep * particles[i].CombinedForce / particles[i].density;
			particles[i].proposedPosition = particles[i].Position + timestep * particles[i].velocity;
		}
	}

	private void HandleCollisions() {
		foreach (Particle p in particles) {
			// If we are colliding with the floor
			if (p.proposedPosition.y < minBound.y + particleRadius) {
				p.proposedPosition.y = minBound.y + particleRadius + Mathf.Epsilon;
				p.velocity.y = boundDamping * p.velocity.y;
			}

			if (p.proposedPosition.y > maxBound.y - particleRadius) {
				p.proposedPosition.y = maxBound.y - particleRadius - Mathf.Epsilon;
				p.velocity.y = boundDamping * p.velocity.y;
			}

			if (p.proposedPosition.x < minBound.x + particleRadius) {
				p.proposedPosition.x = minBound.x + particleRadius + Mathf.Epsilon;
				p.velocity.x = boundDamping * p.velocity.x;
			}

			if (p.proposedPosition.x > maxBound.x - particleRadius) {
				p.proposedPosition.x = maxBound.x - particleRadius - Mathf.Epsilon;
				p.velocity.x = boundDamping * p.velocity.x;
			}

			if (p.proposedPosition.z < minBound.z + particleRadius) {
				p.proposedPosition.z = minBound.z + particleRadius + Mathf.Epsilon;
				p.velocity.z = boundDamping * p.velocity.z;
			}

			if (p.proposedPosition.z > maxBound.z - particleRadius) {
				p.proposedPosition.z = maxBound.z - particleRadius - Mathf.Epsilon;
				p.velocity.z = boundDamping * p.velocity.z;
			}
		}
	}

	public void CommitProposedPosition() {
		foreach (Particle p in particles) {
			p.Position = p.proposedPosition;
			// p.velocity = 0.99f * p.velocity;
		}
	}
}
