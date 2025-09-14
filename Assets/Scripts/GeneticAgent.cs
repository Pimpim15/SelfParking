using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using System;

public class GeneticAgent : MonoBehaviour
{/*
    [SerializeField] private GameObject agentPrefab;
    [SerializeField] private Transform target;
    [SerializeField] private int populationSize = 100;
    [SerializeField] private int generation = 0;
    [SerializeField] private float simulationTime = 30f;
    [SerializeField] private float timeScale = 5f;
    [SerializeField] private float mutationRate = 5f;
    [SerializeField] private float elitePercentage = 0.1f;

    private List<GameObject> agents = new List<GameObject>();
    private float timer = 0f;
    private string timestamp;

    void Start()
    {
        Time.timeScale = timeScale;
        timestamp = DateTime.Now.ToString("dd-MM-yyyy_HH-mm");
        InitializePopulation();
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= simulationTime)
        {
            EvaluateAgents();
            Selection();
            Crossover();
            Mutation();
            NextGeneration();
            timer = 0f;
        }
    }

    void InitializePopulation()
    {
        for (int i = 0; i < populationSize; i++)
        {
            GameObject agent = Instantiate(agentPrefab);
            DNA dna = agent.AddComponent<DNA>();
            agent.GetComponent<DNA>().Initialize(target);
            RandomizeAgentPosition(agent);
            agents.Add(agent);
        }
    }

    void RandomizeAgentPosition(GameObject agent)
    {
        float randPosX = UnityEngine.Random.Range(-4f, 4f);
        float randPosZ = UnityEngine.Random.Range(-7f, -3f);
        float randRotY = UnityEngine.Random.Range(-180f, 180f);

        Vector3 position = new Vector3(randPosX, 0.3f, randPosZ);
        Quaternion rotation = Quaternion.Euler(0, randRotY, 0);

        agent.transform.position = position;
        agent.transform.rotation = rotation;
    }

    void EvaluateAgents()
    {
        foreach (GameObject agent in agents)
        {
            DNA dna = agent.GetComponent<DNA>();
            dna.CalculateFitness();
        }
    }

    void Selection()
    {
        agents = agents.OrderByDescending(a => a.GetComponent<DNA>().fitness).ToList();
    }

    void Crossover()
    {
        int eliteCount = Mathf.RoundToInt(populationSize * elitePercentage);
        List<GameObject> newAgents = new List<GameObject>();

        // Preservar agentes de elite
        for (int i = 0; i < eliteCount; i++)
        {
            GameObject eliteAgent = Instantiate(agentPrefab);
            DNA dna = eliteAgent.AddComponent<DNA>();
            dna.Initialize(target);
            eliteAgent.GetComponent<DNA>().CopyFrom(agents[i].GetComponent<DNA>());
            RandomizeAgentPosition(eliteAgent);
            newAgents.Add(eliteAgent);
        }

        // Gerar descendentes
        while (newAgents.Count < populationSize)
        {
            DNA parentA = agents[UnityEngine.Random.Range(0, eliteCount)].GetComponent<DNA>();
            DNA parentB = agents[UnityEngine.Random.Range(0, eliteCount)].GetComponent<DNA>();

            GameObject offspring = Instantiate(agentPrefab);
            DNA offspringDNA = offspring.AddComponent<DNA>();
            offspringDNA.Initialize(target);
            offspringDNA.Crossover(parentA, parentB);
            RandomizeAgentPosition(offspring);
            newAgents.Add(offspring);
        }

        // Destruir agentes antigos
        foreach (GameObject agent in agents)
        {
            Destroy(agent);
        }

        agents = newAgents;
    }

    void Mutation()
    {
        foreach (GameObject agent in agents)
        {
            if (UnityEngine.Random.Range(0f, 100f) < mutationRate)
            {
                DNA dna = agent.GetComponent<DNA>();
                dna.Mutate();
            }
        }
    }

    void NextGeneration()
    {
        generation++;
        // Salvar dados ou estatísticas se necessário
    }*/
}
