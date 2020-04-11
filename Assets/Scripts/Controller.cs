using System.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

public class Controller : MonoBehaviour {
	public GameObject particlePrefab;
	private Particle[] particles;
	private float particleRadius;
	Kernel kernel;

	[Header("Spawn settings")]
	public Vector3Int cubeDimensions;
	public Vector3 minBounds;
	public Vector3 maxBounds;
	public float particleSpacing;
	public float timestep;

	[Header("Simulation settings")]
	public float smoothingRadius;
	public float boundDamping = -0.5f;
	private float gravityMultiplier;

	[Header("Psysical constants")]
	public float particleMass;
	public float restDensity;
	public float viscosity;
	public float gasConstant;
	public Vector3 external;
	private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

	// This should only show up in jobs branch

	void Start() {
		particleRadius = particlePrefab.transform.localScale.x / 2.0f;
		InstantiateParticles();
		gravityMultiplier = 10 * (1.0f / timestep);
		kernel = new Kernel(smoothingRadius);
	}

	private void Update() {
		
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
						x * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f),
						4.0f + y * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f),
						z * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f)),
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
				particles[i].density += particleMass * kernel.Poly6(particles[i].Position - particles[j].Position);
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
				particles[i].pressureForce -= particleMass * ((particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density)) * kernel.SpikyGradient(particles[i].Position - particles[j].Position);

				// Calculate the viscosity force
				// Equations (14) and (22) in Muller et al. (2003)
				particles[i].viscosityForce += particleMass * viscosity * ((particles[j].velocity - particles[i].velocity) / particles[j].density) * kernel.ViscosityLaplacian(particles[i].Position - particles[j].Position);
			}

			// Evaluate gravity and external force at particle location
			// The force is scaled by the density instead of the mass
			particles[i].externalForce = gravityMultiplier * (external + gravity) * particles[i].density;
		}
	}

	// Standard Euler integration
	private void Integrate() {
		for (int i = 0; i < particles.Length; i++) {
			particles[i].velocity += timestep * particles[i].CombinedForce / particles[i].density;
			particles[i].proposedPosition = particles[i].Position + timestep * particles[i].velocity;
		}
	}

	private void HandleCollisions() {
		foreach (Particle p in particles) {
			if (p.proposedPosition.y < minBounds.y + particleRadius) {
				p.proposedPosition.y = minBounds.y + particleRadius + Mathf.Epsilon;
				p.velocity.y = boundDamping * p.velocity.y;
			}

			if (p.proposedPosition.y > maxBounds.y - particleRadius) {
				p.proposedPosition.y = maxBounds.y - particleRadius - Mathf.Epsilon;
				p.velocity.y = boundDamping * p.velocity.y;
			}

			if (p.proposedPosition.x < minBounds.x + particleRadius) {
				p.proposedPosition.x = minBounds.x + particleRadius + Mathf.Epsilon;
				p.velocity.x = boundDamping * p.velocity.x;
			}

			if (p.proposedPosition.x > maxBounds.x - particleRadius) {
				p.proposedPosition.x = maxBounds.x - particleRadius - Mathf.Epsilon;
				p.velocity.x = boundDamping * p.velocity.x;
			}

			if (p.proposedPosition.z < minBounds.z + particleRadius) {
				p.proposedPosition.z = minBounds.z + particleRadius + Mathf.Epsilon;
				p.velocity.z = boundDamping * p.velocity.z;
			}

			if (p.proposedPosition.z > maxBounds.z - particleRadius) {
				p.proposedPosition.z = maxBounds.z - particleRadius - Mathf.Epsilon;
				p.velocity.z = boundDamping * p.velocity.z;
			}
		}
	}

	public void CommitProposedPosition() {
		foreach (Particle p in particles) {
			p.Position = p.proposedPosition;
		}
	}
}
