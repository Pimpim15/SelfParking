using UnityEngine;
using System.Collections.Generic;
using System;

public class DNA : MonoBehaviour
{/*
    public NeuralNetwork nn;
    public float fitness = 0f;
    private Transform target;
    private Rigidbody rb;
    private Renderer agentRenderer;
    private float initialDistance;

    // Parâmetros do veículo
    private float maxSteeringAngle = 30f;
    private float maxSpeed = 1f; // Reduzido para limitar a velocidade
    private float acceleration = 0.5f; // Reduzido para limitar a aceleração
    private float deceleration = 0.5f; // Reduzido para limitar a desaceleração

    private float steeringAngle = 0f;
    private float currentSpeed = 0f;

    private bool isParked = false;

    public void Initialize(Transform target)
    {
        this.target = target;
        rb = GetComponent<Rigidbody>();
        agentRenderer = GetComponent<Renderer>();

        if (rb == null)
        {
            Debug.LogError("Rigidbody não encontrado no agente.");
        }
        else
        {
            rb.isKinematic = false;
            rb.useGravity = false;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative; // Ajustado
        }

        // Definir o formato da rede neural
        int numSensors = 5; // Defina o número de sensores
        int inputNodes = 6 + numSensors; // 6 entradas básicas + número de sensores
        int[] hiddenNodes = new int[] { 16, 8 }; // Duas camadas ocultas
        int outputNodes = 2; // Acelerador e direção

        nn = new NeuralNetwork(inputNodes, hiddenNodes, outputNodes);
        initialDistance = Vector3.Distance(transform.position, target.position);
    }

    void Update()
    {
        if (!isParked)
        {
            double[] inputs = GetInputs();
            double[] outputs = nn.Predict(inputs);

            ApplyControls(outputs);
            CalculateFitness();
            CheckParkingStatus();
        }
    }
    double[] GetInputs()
    {
        List<double> inputs = new List<double>();

        if (target == null)
        {
            Debug.LogError("Target não está definido no DNA.");
            return new double[6 + 5]; // Retorna um array padrão ou trata o erro apropriadamente
        }

        if (initialDistance == 0)
        {
            initialDistance = Vector3.Distance(transform.position, target.position);
            if (initialDistance == 0)
                initialDistance = 0.0001f; // Evitar divisão por zero
        }

        // 1. Distância normalizada ao alvo
        float currentDistance = Vector3.Distance(transform.position, target.position);
        inputs.Add(currentDistance / initialDistance);

        // 2. Posição relativa ao alvo
        Vector3 relativePosition = transform.InverseTransformPoint(target.position);
        inputs.Add(relativePosition.x / initialDistance);
        inputs.Add(relativePosition.z / initialDistance);

        // 3. Ângulo para o alvo
        Vector3 directionToTarget = (target.position - transform.position).normalized;
        float angleToTarget = Vector3.Angle(transform.forward, directionToTarget) / 180f;
        inputs.Add(angleToTarget);

        // 4. Velocidade atual normalizada
        inputs.Add(currentSpeed / maxSpeed);

        // 5. Ângulo de direção normalizado
        inputs.Add(steeringAngle / maxSteeringAngle);

        // 6. Sensores de proximidade (raycasts)
        int numSensors = 5; // Defina o número de sensores
        float sensorRange = 10f;
        for (int i = 0; i < numSensors; i++)
        {
            float sensorAngle = -90f + (i * 45f); // Ajuste o ângulo conforme necessário
            Quaternion rotation = Quaternion.Euler(0, sensorAngle, 0);
            Vector3 direction = rotation * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, sensorRange))
            {
                inputs.Add(hit.distance / sensorRange);
            }
            else
            {
                inputs.Add(1f); // Nenhum obstáculo detectado
            }
        }

        return inputs.ToArray();
    }

    void ApplyControls(double[] outputs)
    {
        float throttle = Mathf.Clamp((float)outputs[0], -1f, 1f);
        float steerInput = Mathf.Clamp((float)outputs[1], -1f, 1f);

        // Ajustar velocidade
        if (throttle > 0)
        {
            currentSpeed += throttle * acceleration * Time.deltaTime;
        }
        else if (throttle < 0)
        {
            currentSpeed += throttle * deceleration * Time.deltaTime;
        }
        else
        {
            currentSpeed -= Mathf.Sign(currentSpeed) * deceleration * Time.deltaTime;
        }
        currentSpeed = Mathf.Clamp(currentSpeed, -maxSpeed, maxSpeed);

        // Ajustar ângulo de direção
        steeringAngle += steerInput * maxSteeringAngle * Time.deltaTime;
        steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

        // Movimentar agente usando Rigidbody
        float turnRadius = steeringAngle == 0 ? float.MaxValue : (maxSteeringAngle / Mathf.Sin(Mathf.Deg2Rad * steeringAngle));
        float rotationAngle = (currentSpeed / turnRadius) * Mathf.Rad2Deg * Time.deltaTime;

        Quaternion deltaRotation = Quaternion.Euler(0f, rotationAngle, 0f);
        rb.MoveRotation(rb.rotation * deltaRotation);

        Vector3 movement = transform.forward * currentSpeed * Time.deltaTime;

        // Verificar colisão antes de mover
        if (!Physics.Raycast(rb.position, transform.forward, movement.magnitude + 0.1f))
        {
            rb.MovePosition(rb.position + movement);
        }
        else
        {
            // Colisão detectada, aplicar lógica de resposta
            currentSpeed = 0;
            fitness -= 1f;
            //agentRenderer.material.color = Color.red;
        }
    }
    public void CalculateFitness()
    {
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        fitness = (initialDistance - distanceToTarget) / initialDistance;

        // Adicionar alinhamento
        float alignment = Vector3.Dot(transform.forward.normalized, target.forward.normalized);
        fitness += alignment;
    }

    void CheckParkingStatus()
    {
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        bool isAligned = Vector3.Dot(transform.forward.normalized, target.forward.normalized) > 0.9f;

        if (distanceToTarget < 1f && isAligned)
        {
            isParked = true;
            //agentRenderer.material.color = Color.green;
            fitness += 1f;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            // Aplicar penalidade, mas não parar o agente
            fitness -= 1f;
            //agentRenderer.material.color = Color.red;
        }
    }

    public void Mutate()
    {
        nn.Mutate();
    }

    public void CopyFrom(DNA otherDNA)
    {
        nn = NeuralNetwork.Copy(otherDNA.nn);
    }

    public void Crossover(DNA parentA, DNA parentB)
    {
        nn = NeuralNetwork.Crossover(parentA.nn, parentB.nn);
    }*/
}
