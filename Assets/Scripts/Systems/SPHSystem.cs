using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

public class SPHSystem : SystemBase {
	EntityQuery particleGroup;

	protected override void OnCreate() {
		particleGroup = GetEntityQuery(typeof(Translation), typeof(ParticleData));
	}

	protected override void OnUpdate() {
		NativeArray<Entity> entities = particleGroup.ToEntityArray(Allocator.TempJob);
		NativeArray<Translation> positions = particleGroup.ToComponentDataArray<Translation>(Allocator.TempJob);

		JobHandle densityViscosityJob = Entities.ForEach((Entity entity, ref ParticleData particleData, in Translation translation) => {
			particleData.density = 0.0f;

			for (int i = 0; i < positions.Length; i++) {
				particleData.density += Controller.particleMass * Kernels.Poly6(translation.Value - positions[i].Value, Controller.smoothingRadius);
			}

			particleData.pressure = math.max(0.0f, Controller.gasConstant * (particleData.density - Controller.restDensity));
		}).ScheduleParallel(this.Dependency);

		densityViscosityJob.Complete();
		NativeArray<ParticleData> particles = particleGroup.ToComponentDataArray<ParticleData>(Allocator.TempJob);

		JobHandle calculateForceJob = Entities.ForEach((Entity entity, ref ParticleData particleData, in Translation translation) => {
			particleData.pressureForce = float3.zero;
			particleData.viscosityForce = float3.zero;

			for (int j = 0; j < positions.Length; j++) {
				if (entity.Index == entities[j].Index) { continue; }

				particleData.pressureForce -= Controller.particleMass * ((particleData.pressure + particles[j].pressure) / (2.0f * particles[j].density)) * Kernels.SpikyGradient(translation.Value - positions[j].Value, Controller.smoothingRadius);
				particleData.viscosityForce += Controller.particleMass * Controller.viscosity * ((particles[j].velocity - particleData.velocity) / particles[j].density) * Kernels.ViscosityLaplacian(translation.Value - positions[j].Value, Controller.smoothingRadius);
			}

			particleData.gravityForce = Controller.gravityMultiplier * new float3(0f, -9.81f, 0f) * particleData.density;
		}).ScheduleParallel(densityViscosityJob);

		JobHandle integrateJob = Entities.ForEach((ref ParticleData particleData, in Translation translation) => {
			float3 combinedForce = particleData.gravityForce + particleData.pressureForce + particleData.viscosityForce;
			particleData.velocity = Controller.timestep * combinedForce / particleData.density;
			particleData.proposedPosition = translation.Value + Controller.timestep * particleData.velocity;
		}).ScheduleParallel(calculateForceJob);

		JobHandle collisionsJob = Entities.ForEach((ref ParticleData particleData) => {
			if (particleData.proposedPosition.y < Controller.minBounds.y + 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.y = Controller.minBounds.y + 0.5f * Controller.particleSpacing;
				particleData.velocity.y = Controller.boundDamping * particleData.velocity.y;
			}

			if (particleData.proposedPosition.y > Controller.maxBounds.y - 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.y = Controller.maxBounds.y - 0.5f * Controller.particleSpacing;
				particleData.velocity.y = Controller.boundDamping * particleData.velocity.y;
			}

			if (particleData.proposedPosition.x < Controller.minBounds.x + 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.x = Controller.minBounds.x + 0.5f * Controller.particleSpacing;
				particleData.velocity.x = Controller.boundDamping * particleData.velocity.x;
			}

			if (particleData.proposedPosition.x > Controller.maxBounds.x - 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.x = Controller.maxBounds.x - 0.5f * Controller.particleSpacing;
				particleData.velocity.x = Controller.boundDamping * particleData.velocity.x;
			}

			if (particleData.proposedPosition.z < Controller.minBounds.z + 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.z = Controller.minBounds.z + 0.5f * Controller.particleSpacing;
				particleData.velocity.z = Controller.boundDamping * particleData.velocity.z;
			}

			if (particleData.proposedPosition.z > Controller.maxBounds.z - 0.5f * Controller.particleSpacing) {
				particleData.proposedPosition.z = Controller.maxBounds.z - 0.5f * Controller.particleSpacing;
				particleData.velocity.z = Controller.boundDamping * particleData.velocity.z;
			}
		}).ScheduleParallel(integrateJob);

		JobHandle commitProposedJob = Entities.ForEach((ref ParticleData particleData, ref Translation translation) => {
			translation.Value = particleData.proposedPosition;
		}).ScheduleParallel(collisionsJob);

		commitProposedJob.Complete();

		entities.Dispose();
		positions.Dispose();
		particles.Dispose();
	}
}
