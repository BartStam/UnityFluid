using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

public class Controller : MonoBehaviour {
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

	void Start() {
		InstantiateEntities();
		particleRadius = gameObjectPrefab.transform.localScale.x / 2.0f;
	}

	private void InstantiateEntities() {
		World defaultWorld = World.DefaultGameObjectInjectionWorld;
		EntityManager entityManager = defaultWorld.EntityManager;

		GameObjectConversionSettings settings = GameObjectConversionSettings.FromWorld(defaultWorld, null);
		entityPrefab = GameObjectConversionUtility.ConvertGameObjectHierarchy(gameObjectPrefab, settings);

		for (int x = 0; x < cuboidDimensions.x; x++) {
			for (int y = 0; y < cuboidDimensions.y; y++) {
				for (int z = 0; z < cuboidDimensions.z; z++) {
					Entity e = entityManager.Instantiate(entityPrefab);
					entityManager.SetComponentData(e, new Translation { Value = new float3(
						x * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f),
						4.0f + y * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f),
						z * particleSpacing + UnityEngine.Random.Range(-0.2f, 0.2f))
					});
				}
			}
		}
	}
}
