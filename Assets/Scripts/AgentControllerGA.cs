using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;
using Random = UnityEngine.Random;

public class AgentControllerGA : Agent
{
    [SerializeField] private GameObject agent;
    [SerializeField] private Transform target;
    [SerializeField] private Transform frontWall;
    [SerializeField] private float steeringAngle = 0f; // Ângulo de direção atual
    [SerializeField] private float steeringSpeed = 50f; // Velocidade em que o ângulo de direção muda
    [SerializeField] private float maxSteeringAngle = 30f; // Máximo ângulo de direção permitido
    [SerializeField] private float minTurnRadius = 3f; // Raio de giro
    [SerializeField] private float episodeDuration = 20f; // Duração do episódio em segundos
    [SerializeField] private float rewardScalingFactor = 1f;
    [SerializeField] private float moveSpeedAdjusted = 0f;
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float episode = 0f;
    [SerializeField] private float episodeTimer = 0f;
    [SerializeField] private float lastScore = 0f;
    [SerializeField] private float meanScore = 0f;
    //[SerializeField] private float instantScore = 0f;
    //[SerializeField] private float distanceTarget = 0f;
    //private float maxX = 17;
    //private float minX = -17;
    //private float maxZ = 17;
    private float minZ = -17;
    //private int populationSize = 50;

    void Start()
    {
        episodeTimer = 0f; // Reinicia o contador de tempo no início de cada episódio
        steeringAngle = 0f;
        moveSpeedAdjusted = 0f;

        int randPosX = Random.Range(-3, 4); // Posição X aleatória
        //int randPosZ = Random.Range(-15, -12); // Posição Z aleatória
        float randRotY = Random.Range(-30, 31); // Rotação Y aleatória

        transform.localPosition = new Vector3(randPosX, 0.5f, -16f);
        transform.localRotation = Quaternion.Euler(0, randRotY, 0);

        int spotSide = 1;//Random.Range(0, 2);
        int spotPosition = Random.Range(0, 7);

        float spotX = spotSide == 0 ? -9.2f : 9.2f;
        float spotZ = 16 - (spotPosition * 3.28f);
        float spotRotation = spotSide == 0 ? 180 : 0;

        target.localPosition = new Vector3(spotX, 0.05f, spotZ);
        target.localRotation = Quaternion.Euler(0, spotRotation, 270);

        episode++;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //float normalizedPosX = (transform.localPosition.x - minX) / (maxX - minX) * 2 - 1;
        //float normalizedPosZ = (transform.localPosition.z - minZ) / (maxZ - minZ) * 2 - 1;
        // Converte o ?ngulo de rota??o de graus para radianos
        //float angleInRadians = transform.localRotation.eulerAngles.y * Mathf.Deg2Rad;
        //float sinAngle = Mathf.Sin(angleInRadians);
        //float cosAngle = Mathf.Cos(angleInRadians);
        float carDirection = Vector3.Dot(transform.forward, Vector3.forward);
        Vector2 carPosition = new Vector2(transform.localPosition.x, transform.localPosition.z);

        //sensor.AddObservation(normalizedPosX);
        //sensor.AddObservation(normalizedPosZ);
        //sensor.AddObservation(sinAngle);
        //sensor.AddObservation(cosAngle);
        sensor.AddObservation(carPosition);
        sensor.AddObservation(carDirection);

        //normalizedPosX = (target.localPosition.x - minX) / (maxX - minX) * 2 - 1;
        //normalizedPosZ = (target.localPosition.z - minZ) / (maxZ - minZ) * 2 - 1;
        //angleInRadians = target.localRotation.eulerAngles.y * Mathf.Deg2Rad;
        //sinAngle = Mathf.Sin(angleInRadians);
        //cosAngle = Mathf.Cos(angleInRadians);
        float spotAllign = Math.Abs(Vector3.Dot(transform.forward, target.forward));
        Vector2 targetPosition = new Vector2(target.localPosition.x, target.localPosition.z);

        //sensor.AddObservation(normalizedPosX);
        //sensor.AddObservation(normalizedPosZ);
        //sensor.AddObservation(sinAngle);
        //sensor.AddObservation(cosAngle);
        sensor.AddObservation(targetPosition);
        sensor.AddObservation(spotAllign);

        //float normalizedSteeringAngle = (steeringAngle - (-2 * maxSteeringAngle)) / (maxSteeringAngle - (-2 * maxSteeringAngle)) * 2 - 1;

        sensor.AddObservation(steeringAngle);
        //sensor.AddObservation(moveSpeedAdjusted);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float move = actions.ContinuousActions[0];
        float steerInput = actions.ContinuousActions[1];

        steeringAngle += steerInput * steeringSpeed * Time.deltaTime;
        steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

        moveSpeedAdjusted = move * Time.deltaTime * moveSpeed;

        if (steeringAngle != 0)
        {
            float turnRadius = (maxSteeringAngle / steeringAngle) * minTurnRadius;
            //float rotationAngle = (moveSpeedAdjusted / minTurnRadius) * Mathf.Rad2Deg * (steeringAngle / maxSteeringAngle);
            float rotationAngle = (moveSpeedAdjusted / turnRadius) * Mathf.Rad2Deg;

            transform.Rotate(0, rotationAngle, 0);
        }

        transform.Translate(Vector3.forward * moveSpeedAdjusted);

        //instantScore = CalculateReward();
        //distanceTarget = Vector3.Distance(transform.position, target.position);

        // Atualiza o contador de tempo
        episodeTimer += Time.deltaTime;
        if (episodeTimer > episodeDuration)
        {
            // Calcula a recompensa com base na proximidade e orientação
            float reward = CalculateReward();
            Finish(reward); // Encerra o episódio por tempo
        }
    }

    private void Finish(float reward)
    {
        lastScore = reward;

        if (episode == 1)
            meanScore = lastScore;

        meanScore = ((episode * meanScore) + reward) / (episode + 1);
        AddReward(reward);
        EndEpisode();
    }

    private float CalculateReward()
    {
        float maxDistance = 30;
        float zReward = (1 - ((target.position.z - transform.localPosition.z) / (target.position.z - minZ))) * rewardScalingFactor;
        //float distanceSquared = Mathf.Pow(Vector3.Distance(transform.position, target.position), 2);
        float normalizedDistance = Vector3.Distance(transform.position, target.position) / maxDistance; // Isso vai de 0 a 1 para a distância máxima
        float proximityReward = (-normalizedDistance) * rewardScalingFactor; // rewardScalingFactor ajusta a escala da recompensa
        //float proximityReward = (float)(10 - (Math.Pow(Vector3.Distance(transform.position, target.position),2)));

        return proximityReward + zReward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Vertical"); // Movimento para frente e para trás
        continuousActions[1] = Input.GetAxisRaw("Horizontal"); // Mudança no ângulo de direção com "a" e "d"
    }

    private void OnTriggerEnter(Collider other)
    {
        float collisionReward = -1f;

        if (other.gameObject.CompareTag("Car"))
        {
            collisionReward = CalculateReward() - (0.2f * rewardScalingFactor);
        }
        else if (other.gameObject.CompareTag("Wall"))
        {
            collisionReward = CalculateReward() - (transform.position.z < -15 ? 1f : 0.5f * rewardScalingFactor);
        }
        else if (other.gameObject.CompareTag("Spot"))
        {
            bool isAligned = Math.Abs(Vector3.Dot(transform.forward, target.forward)) < 0.1;
            bool isClose = Vector3.Distance(transform.position, target.position) < 1;

            if (isAligned && isClose)
            {
                float remainderTime = episodeDuration - episodeTimer;
                float timeRewardNormalized = episodeDuration / remainderTime;
                collisionReward = 1 + timeRewardNormalized + (CalculateReward() * rewardScalingFactor);
            }
            else
            {
                return;
            }
        }

        Finish(collisionReward);
    }

    void OnDrawGizmos()
    {
        // Verifica se há um ângulo de direção para evitar divisão por zero
        if (Mathf.Abs(steeringAngle) > 0)
        {
            // Calcula um raio de giro dinâmico baseado no ângulo de direção
            // Aqui assumimos que quanto maior o ângulo, menor o raio, simulando um giro mais fechado
            float dynamicTurnRadius = minTurnRadius * (maxSteeringAngle / Mathf.Abs(steeringAngle));

            // Calcula o centro de rotação dinâmico com base no ângulo de direção
            Vector3 dynamicRotationCenter = transform.position + (transform.right * dynamicTurnRadius * (steeringAngle > 0 ? 1 : -1));

            // Desenha o centro de rotação
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(dynamicRotationCenter, 0.3f); // Ajuste o tamanho da esfera conforme necessário

            // Desenha a circunferência de giro dinâmica
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(dynamicRotationCenter, dynamicTurnRadius);
        }
    }


}
