using UnityEngine;
using System.Collections.Generic;
using Assets.Scripts;

public class AgentController : MonoBehaviour
{
    public NeuralNetwork Network { get; private set; }
    public float Fitness { get; private set; }
    public bool HasFinished { get; private set; }

    [Header("Sensor Settings")]
    [SerializeField] private float sensorLength = 10f;
    [SerializeField] private LayerMask obstacleLayer;

    [Header("Car Settings")]
    [SerializeField] private float maxSteerAngle = 30f; // �ngulo m�ximo de dire��o
    [SerializeField] private float motorForce = 1500f;  // For�a do motor
    [SerializeField] private float maxSpeed = 20f;      // Velocidade m�xima
    [SerializeField] private float wheelBase = 2.5f;    // Dist�ncia entre os eixos do ve�culo

    [Header("Time Settings")]
    [SerializeField] private float maxTime = 30f;       // Tempo m�ximo em segundos
    private float elapsedTime = 0f;

    private Vector3 startPosition;
    private Quaternion startRotation;
    private Rigidbody rb;

    private float steeringAngle;
    private float currentSpeed;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    public void InitializeNeuralNetwork()
    {
        int inputNodes = 5; // Exemplo: 5 sensores
        List<int> hiddenNodes = new List<int> { 10, 10 }; // Duas camadas ocultas com 10 n�s cada
        int outputNodes = 2; // Acelera��o e dire��o

        Network = new NeuralNetwork(inputNodes, hiddenNodes, outputNodes);
    }

    public void SetNeuralNetwork(NeuralNetwork network)
    {
        Network = NeuralNetwork.Copy(network);
    }

    void FixedUpdate()
    {
        if (HasFinished)
            return;

        elapsedTime += Time.fixedDeltaTime;

        // Verifica se o tempo m�ximo foi excedido
        if (elapsedTime >= maxTime)
        {
            HasFinished = true;
            return;
        }

        // Coleta entradas dos sensores
        double[] inputs = GetSensorReadings();

        // Obt�m as sa�das da rede neural
        double[] outputs = Network.FeedForward(inputs);

        // Aplica as a��es com base nas sa�das
        ApplyActions(outputs);

        // Avalia o fitness do agente
        EvaluateFitness();

        // Verifica se o agente atingiu o objetivo ou colidiu
        CheckCompletion();
    }

    double[] GetSensorReadings()
    {
        double[] inputs = new double[5];

        // Sensor frontal
        inputs[0] = CastSensor(transform.forward);

        // Sensor direito
        inputs[1] = CastSensor(transform.right);

        // Sensor esquerdo
        inputs[2] = CastSensor(-transform.right);

        // Sensor frontal direito
        inputs[3] = CastSensor((transform.forward + transform.right).normalized);

        // Sensor frontal esquerdo
        inputs[4] = CastSensor((transform.forward - transform.right).normalized);

        return inputs;
    }

    double CastSensor(Vector3 direction)
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, direction, out hit, sensorLength, obstacleLayer))
        {
            return 1 - (hit.distance / sensorLength);
        }
        else
        {
            return 0;
        }
    }

    void ApplyActions(double[] outputs)
    {
        float accelerationInput = Mathf.Clamp((float)outputs[0], -1f, 1f); // Entrada de acelera��o (-1 a 1)
        float steeringInput = Mathf.Clamp((float)outputs[1], -1f, 1f);     // Entrada de dire��o (-1 a 1)

        // Calcula o �ngulo de dire��o
        steeringAngle = maxSteerAngle * steeringInput;

        // Atualiza a velocidade atual
        currentSpeed = rb.velocity.magnitude;

        // Limita a velocidade m�xima
        if (currentSpeed < maxSpeed)
        {
            Vector3 force = transform.forward * accelerationInput * motorForce * Time.deltaTime;
            rb.AddForce(force, ForceMode.Force);
        }

        // Simula a dire��o alterando a orienta��o do ve�culo
        float turnRadius = Mathf.Abs(currentSpeed) > 0.1f ? wheelBase / Mathf.Tan(Mathf.Deg2Rad * steeringAngle) : Mathf.Infinity;
        float angularVelocity = currentSpeed / turnRadius;

        if (!float.IsNaN(angularVelocity) && !float.IsInfinity(angularVelocity))
        {
            rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, angularVelocity * Mathf.Rad2Deg * Time.deltaTime, 0f));
        }
    }

    void EvaluateFitness()
    {
        // Encontra a vaga dispon�vel mais pr�xima
        GameObject[] targets = GameObject.FindGameObjectsWithTag("Target");
        float minDistance = float.MaxValue;

        foreach (var target in targets)
        {
            float distance = Vector3.Distance(transform.position, target.transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
            }
        }

        Fitness = 1 / (minDistance + 1f); // Evita divis�o por zero
    }


    void CheckCompletion()
    {
        // Verifica se o agente chegou ao target
        Collider[] colliders = Physics.OverlapSphere(transform.position, 1f);
        foreach (var collider in colliders)
        {
            if (collider.CompareTag("Target"))
            {
                HasFinished = true;
                Fitness += 10f; // B�nus por completar o objetivo
                return;
            }
        }

        // Verifica se o agente capotou ou caiu
        if (Mathf.Abs(transform.eulerAngles.z) > 45f || transform.position.y < -1f)
        {
            HasFinished = true;
        }
    }


    public void ResetAgent(Vector3 position, Quaternion rotation)
    {
        transform.position = position;
        transform.rotation = rotation;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        Fitness = 0f;
        HasFinished = false;
        elapsedTime = 0f;
    }
}
