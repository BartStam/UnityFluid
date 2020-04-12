using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct ParticleData : IComponentData {
	public float3 velocity;
	public float3 proposedPosition;

	public float density;
	public float pressure;

	public float3 viscosityForce;
	public float3 gravityForce;
	public float3 pressureForce;
	
}
