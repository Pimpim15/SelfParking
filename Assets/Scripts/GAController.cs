using Assets.Scripts;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

public class GAController : MonoBehaviour
{
    [Header("GA")]
    [SerializeField] int popSize = 60, elite = 6;
    [SerializeField] int[] hidden = { 24, 16 };
    [SerializeField] float timeScale = 5f; // Tempo de simulação

    // Novos parâmetros expostos para controlar evolução
    [Header("GA - Selection / Mutation")]
    [SerializeField] int tournamentK = 3;               // tamanho do torneio
    [SerializeField] float baseMutationRate = 0.20f;    // taxa base de mutação (probabilidade de mutar um peso)
    [SerializeField] float baseMutationStd = 0.05f;     // desvio padrão base para ruído Gaussiano
    [SerializeField] float immigrantRate = 0.05f;       // probabilidade de gerar um indivíduo aleatório por filho
    [SerializeField] bool useAdaptiveMutation = true;   // ativa/adapta mutação baseada em diversidade/geração
    [SerializeField] int maxGenerationsForDecay = 2000; // escalonamento temporal para decaimento da mutação

    [Header("Prefabs / Cena")]
    [SerializeField] GameObject carPrefab;
    [SerializeField] private GameObject targetPrefab;          // Prefab da vaga disponível
    [SerializeField] private GameObject obstacleCarPrefab;     // Prefab do carro de obstáculo
    [SerializeField] private LayerMask obstacleMask;

    [Header("Coleta / Export")]
    [SerializeField] bool   logTrajectoriesTop1 = true;
    [SerializeField] int    trajectoryMaxSteps = 800;
    [SerializeField] int    saveTopNNetworks = 3;

    List<CarAgent> population = new List<CarAgent>();
    int generation;
    int agentSerial;

    // Listas para armazenar as vagas
    private List<GameObject> parkingSpots = new List<GameObject>();
    private List<GameObject> obstacleCars = new List<GameObject>();
    private GameObject target; // vaga atual (a ser atribuída a cada agente)

    // Lista das posições e rotações das vagas
    private List<Vector3> parkingSpotsPositions = new List<Vector3>();
    private List<float> parkingSpotsRotations = new List<float>();

    void Awake()
    {
        Time.timeScale = timeScale;
        RunLogger.Init("SelfParkingGA");
        RunLogger.WriteRunConfig(new RunConfig
        {
            popSize = popSize,
            elite = elite,
            hidden = hidden,
            timeScale = timeScale,
            obstacleMask = (int)obstacleMask.value,
            logTrajectoriesTop1 = logTrajectoriesTop1,
            trajectoryMaxSteps = trajectoryMaxSteps,
            saveTopNNetworks = saveTopNNetworks
        });
    }

    void Start() 
    {
        InitializeParkingSpots();
        SetupParkingLot();
        SpawnInitial();
    }

    void Update()
    {
        Time.timeScale = timeScale;

        if (population.All(a => a.Done))
        {
            population = population.OrderByDescending(a => a.Fitness).ToList();

            // Coleta métricas da geração
            var agentsMetrics = population.Select(a => a.GetFinalMetrics()).ToList();

            // Exporta estatísticas / CSV / trajetórias / redes
            ExportGenerationData(agentsMetrics, population);

            InitializeParkingSpots();
            SetupParkingLot();

            // Evolução
            Evolve();
            generation++;
        }
    }

    void InitializeParkingSpots()
    {
        parkingSpotsPositions = new List<Vector3>();
        parkingSpotsRotations = new List<float>();

        int spotQuant = 7; // Quantidade de vagas

        for (int spotSide = 0; spotSide < 2; spotSide++) // 0: Esquerda, 1: Direita
        {
            for (int spotPosition = 0; spotPosition < spotQuant; spotPosition++)
            {
                float spotX = spotSide == 0 ? -11f : 7f;
                float spotZ = 20f - (spotPosition * 3.28f);
                float spotRotation = spotSide == 0 ? 180f : 0f;

                parkingSpotsPositions.Add(new Vector3(spotX, 0.05f, spotZ));
                parkingSpotsRotations.Add(spotRotation);
            }
        }
    }

    void SetupParkingLot()
    {
        // Limpa vagas e obstáculos anteriores
        foreach (var spot in parkingSpots) Destroy(spot);
        parkingSpots.Clear();

        foreach (var car in obstacleCars) Destroy(car);
        obstacleCars.Clear();

        // Define vagas ocupadas e disponíveis
        for (int i = 0; i < parkingSpotsPositions.Count; i++)
        {
            // Somente a 5a vaga da esquerda (index 11) estará desocupada
            bool isOccupied = i != 11;

            if (isOccupied)
            {
                // Adiciona uma pequena variação na posição dos obstáculos
                float positionNoiseX = UnityEngine.Random.Range(-0.5f, 0.5f);
                float positionNoiseZ = UnityEngine.Random.Range(-0.5f, 0.5f);

                GameObject obstacleCar = Instantiate(obstacleCarPrefab, parkingSpotsPositions[i] + new Vector3(positionNoiseX, 0f, positionNoiseZ),
                    Quaternion.Euler(0f, parkingSpotsRotations[i] - 90, 0f));
                obstacleCars.Add(obstacleCar);
                obstacleCar.layer = LayerMask.NameToLayer("Obstacle");
            }
            else
            {
                // Ajusta o offset da vaga
                var spotOffset = new Vector3(0.51f, 0f, 0.21f);

                // Instancia o targetPrefab na vaga disponível
                GameObject parkingSpot = Instantiate(targetPrefab, parkingSpotsPositions[i] + spotOffset, Quaternion.Euler(0f, 0f, 270f));
                parkingSpots.Add(parkingSpot);

                target = parkingSpot;
            }
        }
    }

    void SpawnInitial()
    {
        population.Clear();
        agentSerial = 0;
        generation = 0;

        for (int i = 0; i < popSize; i++)
        {
            var nn = new NeuralNetwork(19, hidden.ToList(), 2);
            population.Add(SpawnCar(nn));
        }
    }

    CarAgent SpawnCar(NeuralNetwork nn)
    {
        var go = Instantiate(carPrefab, RandomSpawn(), Quaternion.identity);
        var agent = go.AddComponent<CarAgent>();

        bool recordTraj = logTrajectoriesTop1; // registro para todos (descartamos depois, se quiser reduzir, use false aqui)
        agent.Init(nn, target, obstacleMask, generation, agentSerial++, recordTraj, trajectoryMaxSteps);
        agent.EpisodeEnded += OnAgentEpisodeEnded;
        return agent;
    }

    void OnAgentEpisodeEnded(CarAgent agent)
    {
        // hook se quiser lógica por agente (atualmente exportamos no fim da geração)
    }

    void Evolve()
    {
        var nextGen = new List<CarAgent>();

        // elitismo efetivo (limitado a uma fração para evitar a perda rápida de diversidade)
        int effectiveElite = Mathf.Clamp(elite, 1, Mathf.Max(1, popSize / 4));
        for (int i = 0; i < effectiveElite; i++)
            nextGen.Add(SpawnCar(NeuralNetwork.Copy(population[i].Net)));

        // estatísticas de fitness
        var fits = population.Select(a => a.Fitness).ToArray();
        float minFit = fits.Min();
        float offset = minFit < 0 ? -minFit + 1e-3f : 0f;
        float fitnessSum = population.Sum(a => a.Fitness + offset);

        float meanFit = fits.Average();
        float stdevFit = (float)Math.Sqrt(fits.Select(x => (x - meanFit) * (x - meanFit)).Average());

        // mutação adaptativa (taxa e desvio)
        float genScale = 1f - Mathf.Clamp01((float)generation / (float)Mathf.Max(1, maxGenerationsForDecay));
        float diversityFactor = 1f / (1f + stdevFit + 1e-6f); // maior quando stdev baixo (baixa diversidade)
        float mutationRate = baseMutationRate;
        float mutationStd = baseMutationStd;

        if (useAdaptiveMutation)
        {
            // adaptações suaves; limites para estabilidade
            mutationRate = Mathf.Clamp(baseMutationRate * (0.5f + genScale * diversityFactor), 0.01f, 0.6f);
            mutationStd  = Mathf.Clamp(baseMutationStd  * (1f + genScale * diversityFactor), 0.001f, 1f);
        }

        // seleção por torneio
        NeuralNetwork TournamentSelect(int k)
        {
            CarAgent best = null;
            for (int i = 0; i < k; i++)
            {
                var candidate = population[UnityEngine.Random.Range(0, population.Count)];
                if (best == null || candidate.Fitness > best.Fitness) best = candidate;
            }
            return best.Net;
        }

        while (nextGen.Count < popSize)
        {
            // imigrante aleatório para manter diversidade
            if (UnityEngine.Random.value < immigrantRate)
            {
                var nnRand = new NeuralNetwork(19, hidden.ToList(), 2);
                nextGen.Add(SpawnCar(nnRand));
                continue;
            }

            var parentA = TournamentSelect(tournamentK);
            var parentB = TournamentSelect(tournamentK);
            var child = NeuralNetwork.Crossover(parentA, parentB);
            child.Mutate(mutationRate, mutationStd);
            nextGen.Add(SpawnCar(child));
        }

        foreach (var a in population) Destroy(a.gameObject);
        population = nextGen;
    }

    Vector3 RandomSpawn() => new Vector3(UnityEngine.Random.Range(2f, 2.5f), 0, UnityEngine.Random.Range(12f, 13f));

    // -------- Export helpers --------

    [Serializable]
    class GenerationSummary
    {
        public int generation;
        public int popSize;
        public int elite;
        public int[] hidden;
        public float timeScale;

        public float bestFitness;
        public float meanFitness;
        public float medianFitness;
        public float stdevFitness;

        public float parkedRate;
        public float collisionRate;
        public float meanTimeAlive;
        public float meanDistance;
        public float meanLateralError;
        public float meanHeadingAlign;
        public float meanDirAlign;

        public string[] savedNetworks;       // paths
        public string   top1TrajectoryPath;  // path
        public string   agentsCsvPath;       // path
    }

    void ExportGenerationData(List<CarAgent.AgentMetrics> metrics, List<CarAgent> agents)
    {
        string genDir = RunLogger.GenFolder(generation);
        Directory.CreateDirectory(genDir);

        // ------- CSV por agente -------
        string csvPath = Path.Combine(genDir, $"gen_{generation:0000}_agents.csv");
        WriteAgentsCsv(csvPath, metrics);

        // ------- estatísticas agregadas -------
        var fit = metrics.Select(m => m.fitness).ToArray();
        float mean = (float)fit.Average();
        float med = (float)fit.OrderBy(x => x).ElementAt(fit.Length / 2);
        float stdev = (float)Math.Sqrt(fit.Select(x => (x - mean) * (x - mean)).Average());

        float parkedRate = (float)metrics.Count(m => m.parked) / metrics.Count;
        float collisionRate = (float)metrics.Count(m => m.collided) / metrics.Count;

        // salvar topN redes
        List<string> saved = new List<string>();
        int topN = Mathf.Min(saveTopNNetworks, agents.Count);
        for (int i = 0; i < topN; i++)
        {
            string nnPath = Path.Combine(genDir, $"gen_{generation:0000}_best_{i+1}NN.json");
            agents[i].Net.SaveToJson(nnPath);
            saved.Add(nnPath);
        }

        // trajetória do melhor
        string trajPath = null;
        if (logTrajectoriesTop1)
        {
            var traj = agents[0].GetTrajectory();
            if (traj != null && traj.Count > 0)
            {
                trajPath = Path.Combine(genDir, $"gen_{generation:0000}_top1_traj.json");
                RunLogger.WriteJsonList(trajPath, traj); // <— aqui
            }
        }

        var summary = new GenerationSummary
        {
            generation = generation,
            popSize = popSize,
            elite = elite,
            hidden = hidden,
            timeScale = timeScale,

            bestFitness = agents[0].Fitness,
            meanFitness = mean,
            medianFitness = med,
            stdevFitness = stdev,

            parkedRate = parkedRate,
            collisionRate = collisionRate,
            meanTimeAlive = (float)metrics.Average(m => m.timeAlive),
            meanDistance = (float)metrics.Average(m => m.distance),
            meanLateralError = (float)metrics.Average(m => m.meanLateralError),
            meanHeadingAlign = (float)metrics.Average(m => m.meanHeadingAlign),
            meanDirAlign = (float)metrics.Average(m => m.meanDirAlign),

            savedNetworks = saved.ToArray(),
            top1TrajectoryPath = trajPath,
            agentsCsvPath = csvPath
        };

        string sumPath = Path.Combine(genDir, $"gen_{generation:0000}_summary.json");
        RunLogger.WriteJson(sumPath, summary);

        Debug.Log($"Gen {generation} | best={agents[0].Fitness:F2} | mean={mean:F2} | parked={parkedRate:P0} | saved {topN} nets");
    }

    void WriteAgentsCsv(string path, List<CarAgent.AgentMetrics> ms)
    {
        using (var sw = new StreamWriter(path, false, System.Text.Encoding.UTF8))
        {
            string[] headers = new[]{
                "generation","agentId","fitness","parked","collided",
                "timeAlive","distance","maxSpeed","meanSpeed","meanAbsSteer","meanAbsThrottle",
                "gateEntries","insideGateTime","meanLateralError","meanHeadingAlign","meanDirAlign","steps"
            };
            sw.WriteLine(string.Join(",", headers));

            foreach (var m in ms)
            {
                string[] row = new[]{
                    m.generation.ToString(),
                    m.agentId.ToString(),
                    m.fitness.ToString(CultureInfo.InvariantCulture),
                    m.parked ? "1":"0",
                    m.collided ? "1":"0",
                    m.timeAlive.ToString(CultureInfo.InvariantCulture),
                    m.distance.ToString(CultureInfo.InvariantCulture),
                    m.maxSpeed.ToString(CultureInfo.InvariantCulture),
                    m.meanSpeed.ToString(CultureInfo.InvariantCulture),
                    m.meanAbsSteer.ToString(CultureInfo.InvariantCulture),
                    m.meanAbsThrottle.ToString(CultureInfo.InvariantCulture),
                    m.gateEntries.ToString(),
                    m.insideGateTime.ToString(CultureInfo.InvariantCulture),
                    m.meanLateralError.ToString(CultureInfo.InvariantCulture),
                    m.meanHeadingAlign.ToString(CultureInfo.InvariantCulture),
                    m.meanDirAlign.ToString(CultureInfo.InvariantCulture),
                    m.steps.ToString()
                };
                sw.WriteLine(string.Join(",", row));
            }
        }
    }

    // --------- Run logger util ---------
    [Serializable]
    public class RunConfig
    {
        public int popSize;
        public int elite;
        public int[] hidden;
        public float timeScale;
        public int obstacleMask;
        public bool logTrajectoriesTop1;
        public int trajectoryMaxSteps;
        public int saveTopNNetworks;
    }

    static class RunLogger
    {
        static string root;
        static string runFolder;

        [Serializable] class Wrapper<T> { public T data; }

        public static void Init(string runLabel)
        {
            root = Path.Combine(Application.persistentDataPath, "SelfParkingLogs");
            Directory.CreateDirectory(root);

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            runFolder = Path.Combine(root, $"{runLabel}_{stamp}");
            Directory.CreateDirectory(runFolder);

            Debug.Log($"Run folder: {runFolder}");
        }

        public static string GenFolder(int gen)
        {
            string g = Path.Combine(runFolder, $"gen_{gen:0000}");
            Directory.CreateDirectory(g);
            return g;
        }

        public static void WriteRunConfig(RunConfig cfg)
        {
            var path = Path.Combine(runFolder, "run_config.json");
            WriteJson(path, cfg);
        }

        public static void WriteJson<T>(string path, T obj)
        {
            // Para objetos normais (não-lista), serialize direto
            string json = JsonUtility.ToJson(obj, prettyPrint: true);
            File.WriteAllText(path, json, System.Text.Encoding.UTF8);
        }

        public static void WriteJsonList<T>(string path, System.Collections.Generic.List<T> list)
        {
            // Envolve a lista para o JsonUtility
            var wrapper = new Wrapper<System.Collections.Generic.List<T>> { data = list };
            string json = JsonUtility.ToJson(wrapper, prettyPrint: true);
            File.WriteAllText(path, json, System.Text.Encoding.UTF8);
        }
    }

    // --------- Avaliar rede salva (opcional para seus testes) ---------
    public void EvaluateSavedNetwork(string jsonPath, int trials = 5)
    {
        var nn = NeuralNetwork.LoadFromJson(jsonPath);
        if (nn == null)
        {
            Debug.LogError($"Falha ao carregar NN: {jsonPath}");
            return;
        }

        // limpa a população atual, spawna “trials” agentes só para avaliação (sem evolução)
        foreach (var a in population) Destroy(a.gameObject);
        population.Clear();
        agentSerial = 0;

        for (int i = 0; i < trials; i++)
            population.Add(SpawnCar(NeuralNetwork.Copy(nn)));

        generation = 0; // rodar como geração 0 isolada
    }
}
