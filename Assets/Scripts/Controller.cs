using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Rendering;
using Unity.Jobs;
using UnityEngine;

public class Controller : MonoBehaviour {
	public GameObject particlePrefab;
	private Particle[] particles;
	private float particleRadius;

	[Header("Spawn settings")]
	public Vector3Int cuboidDimensions;
	public readonly static Vector3 minBounds = new Vector3(-8f, 0f, -8f);
	public readonly static Vector3 maxBounds = new Vector3(8f, 30f, 8f);
	public readonly static float particleSpacing = 0.2f;
	public readonly static float timestep = 0.0004f;

	[Header("Simulation settings")]
	public readonly static float smoothingRadius = 0.2f;
	public readonly static float boundDamping = -0.5f;
	public readonly static float gravityMultiplier = 24000f;

	[Header("Psysical constants")]
	public readonly static float particleMass = 65f;
	public readonly static float restDensity = 500;
	public readonly static float viscosity = 250f;
	public readonly static float gasConstant = 4000f;
	public readonly static Vector3 external = new Vector3(0f, 0f, 0f);
	private readonly static Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

	[Header("ECS")]
	public GameObject gameObjectPrefab;

	private Entity entityPrefab;
	private World defaultWorld;
	private EntityManager entityManager;

	private Entity[] particleEntities;


	void Start() {
		float3 testFloat = new float3(2f, 2f, 0f);

		InstantiateEntities();
		//InstantiateParticles();

		particleRadius = particlePrefab.transform.localScale.x / 2.0f;
	}



	private void InstantiateEntities() {
		defaultWorld = World.DefaultGameObjectInjectionWorld;
		entityManager = defaultWorld.EntityManager;

		GameObjectConversionSettings settings = GameObjectConversionSettings.FromWorld(defaultWorld, null);
		entityPrefab = GameObjectConversionUtility.ConvertGameObjectHierarchy(gameObjectPrefab, settings);

		particleEntities = new Entity[cuboidDimensions.x * cuboidDimensions.y * cuboidDimensions.z];

		for (int x = 0; x < cuboidDimensions.x; x++) {
			for (int y = 0; y < cuboidDimensions.y; y++) {
				for (int z = 0; z < cuboidDimensions.z; z++) {
					Entity e = entityManager.Instantiate(entityPrefab);
					entityManager.SetComponentData(e, new Translation { Value = new float3(
						x * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f),
						4.0f + y * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f),
						z * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f))
					});

					particleEntities[x * cuboidDimensions.y * cuboidDimensions.z + y * cuboidDimensions.z + z] = e;
				}
			}
		}
	}

	void FixedUpdate() {
		//UpdateDensityPressure();
		//UpdateForces();
		//Integrate();
		//HandleCollisions();
		//CommitProposedPosition();
	}

	private void InstantiateParticles() {
		particles = new Particle[cuboidDimensions.x * cuboidDimensions.y * cuboidDimensions.z];

		for (int x = 0; x < cuboidDimensions.x; x++) {
			for (int y = 0; y < cuboidDimensions.y; y++) {
				for (int z = 0; z < cuboidDimensions.z; z++) {
					GameObject p = Instantiate(particlePrefab, new Vector3(
						x * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f),
						4.0f + y * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f),
						z * particleSpacing + UnityEngine.Random.Range(-0.1f, 0.1f)),
						Quaternion.identity);

					particles[x * cuboidDimensions.y * cuboidDimensions.z + y * cuboidDimensions.z + z] = new Particle(p);
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
