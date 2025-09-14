using Assets.Scripts;
using System.Collections.Generic;
using UnityEngine;

public class EnvironmentController : MonoBehaviour
{
    [SerializeField] private GameObject agentPrefab;
    [SerializeField] private GameObject targetPrefab;          // Prefab da vaga disponível
    [SerializeField] private GameObject obstacleCarPrefab;     // Prefab do carro de obstáculo

    [Header("GA Settings")]
    [SerializeField] private int populationSize = 50;
    [SerializeField] private int elitism = 5;
    [SerializeField] private float mutationRate = 0.05f;

    [Header("Agent Spawn Settings")]
    [SerializeField] private Vector2 spawnXRange = new Vector2(-4f, 4f);
    [SerializeField] private Vector2 spawnZRange = new Vector2(-7f, -3f);
    [SerializeField] private Vector2 spawnYRotationRange = new Vector2(-30f, 30f);

    private List<AgentController> agents = new List<AgentController>();
    private int currentGeneration = 0;

    // Listas para armazenar as vagas e obstáculos
    private List<GameObject> parkingSpots = new List<GameObject>();
    private List<GameObject> obstacleCars = new List<GameObject>();

    // Lista das posições e rotações das vagas
    private List<Vector3> parkingSpotsPositions = new List<Vector3>();
    private List<float> parkingSpotsRotations = new List<float>();

    void Start()
    {
        InitializeParkingSpots();
        InitializeAgents();
        SetupParkingLot();
    }

    void InitializeParkingSpots()
    {
        for (int spotSide = 0; spotSide < 2; spotSide++) // 0: Esquerda, 1: Direita
        {
            for (int spotPosition = 0; spotPosition < 7; spotPosition++)
            {
                float spotX = spotSide == 0 ? -9.2f : 9.2f;
                float spotZ = 16f - (spotPosition * 3.28f);
                float spotRotation = spotSide == 0 ? 180f : 0f;

                parkingSpotsPositions.Add(new Vector3(spotX, 0.05f, spotZ));
                parkingSpotsRotations.Add(spotRotation);
            }
        }
    }

    void InitializeAgents()
    {
        for (int i = 0; i < populationSize; i++)
        {
            Vector3 spawnPosition = GetRandomSpawnPosition();
            Quaternion spawnRotation = GetRandomSpawnRotation();

            GameObject agentObj = Instantiate(agentPrefab, spawnPosition, spawnRotation);
            AgentController agent = agentObj.GetComponent<AgentController>();

            // Inicializa a rede neural do agente
            agent.InitializeNeuralNetwork();

            agents.Add(agent);
        }
    }

    void SetupParkingLot()
    {
        // Limpa vagas e obstáculos anteriores
        foreach (var spot in parkingSpots)
        {
            Destroy(spot);
        }
        parkingSpots.Clear();

        foreach (var car in obstacleCars)
        {
            Destroy(car);
        }
        obstacleCars.Clear();

        // Define vagas ocupadas e disponíveis
        for (int i = 0; i < parkingSpotsPositions.Count; i++)
        {
            // Decide aleatoriamente se a vaga estará ocupada
            bool isOccupied = Random.value > 0.5f; // 50% de chance de estar ocupada

            if (isOccupied)
            {
                // Instancia um carro de obstáculo na vaga
                GameObject obstacleCar = Instantiate(obstacleCarPrefab, parkingSpotsPositions[i], Quaternion.Euler(0f, parkingSpotsRotations[i], 0f));
                obstacleCars.Add(obstacleCar);

                // Define a layer do obstáculo para "Obstacle" ou a layer desejada
                obstacleCar.layer = LayerMask.NameToLayer("Obstacle");
            }
            else
            {
                // Instancia o targetPrefab na vaga disponível
                GameObject parkingSpot = Instantiate(targetPrefab, parkingSpotsPositions[i], Quaternion.Euler(0f, parkingSpotsRotations[i], 0f));
                parkingSpots.Add(parkingSpot);

                // Opcional: definir tag ou layer para o target
                parkingSpot.tag = "Target";
            }
        }
    }

    Vector3 GetRandomSpawnPosition()
    {
        float x = Random.Range(spawnXRange.x, spawnXRange.y);
        float z = Random.Range(spawnZRange.x, spawnZRange.y);
        return new Vector3(x, 0f, z);
    }

    Quaternion GetRandomSpawnRotation()
    {
        float yRotation = Random.Range(spawnYRotationRange.x, spawnYRotationRange.y);
        return Quaternion.Euler(0f, yRotation, 0f);
    }

    void Update()
    {
        // Verifica se todos os agentes terminaram suas simulações
        if (AllAgentsFinished())
        {
            EvolveAgents();
            ResetAgents();
            SetupParkingLot(); // Reconfigura o estacionamento para a próxima geração
        }
    }

    bool AllAgentsFinished()
    {
        foreach (var agent in agents)
        {
            if (!agent.HasFinished)
                return false;
        }
        return true;
    }

    void EvolveAgents()
    {
        // Ordena os agentes com base no fitness
        agents.Sort((a, b) => b.Fitness.CompareTo(a.Fitness));

        List<AgentController> newAgents = new List<AgentController>();

        // Preserva os melhores agentes (elitismo)
        for (int i = 0; i < elitism; i++)
        {
            newAgents.Add(agents[i]);
        }

        // Gera novos agentes através de crossover e mutação
        while (newAgents.Count < populationSize)
        {
            AgentController parent1 = SelectAgent();
            AgentController parent2 = SelectAgent();

            NeuralNetwork offspringNN = NeuralNetwork.Crossover(parent1.Network, parent2.Network);
            offspringNN.Mutate(mutationRate);

            Vector3 spawnPosition = GetRandomSpawnPosition();
            Quaternion spawnRotation = GetRandomSpawnRotation();

            GameObject agentObj = Instantiate(agentPrefab, spawnPosition, spawnRotation);
            AgentController agent = agentObj.GetComponent<AgentController>();
            agent.SetNeuralNetwork(offspringNN);

            newAgents.Add(agent);
        }

        // Destrói os agentes antigos
        foreach (var agent in agents)
        {
            if (!newAgents.Contains(agent))
            {
                Destroy(agent.gameObject);
            }
        }

        agents = newAgents;
        currentGeneration++;
    }

    AgentController SelectAgent()
    {
        // Implementa seleção por roleta

        float totalFitness = 0f;
        foreach (var agent in agents)
        {
            totalFitness += agent.Fitness;
        }

        float randomPoint = Random.value * totalFitness;
        float cumulativeFitness = 0f;

        foreach (var agent in agents)
        {
            cumulativeFitness += agent.Fitness;
            if (cumulativeFitness >= randomPoint)
                return agent;
        }

        return agents[Random.Range(0, agents.Count)];
    }

    void ResetAgents()
    {
        foreach (var agent in agents)
        {
            agent.ResetAgent(GetRandomSpawnPosition(), GetRandomSpawnRotation());
        }
    }
}
