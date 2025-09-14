using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using System.IO;

public class GeneticAgentController : MonoBehaviour
{/*
    [SerializeField] private GameObject agent;
    [SerializeField] private GameObject car;
    [SerializeField] private Transform target;
    [SerializeField] private int carCount = 5;
    [SerializeField] private Transform environmentLocation;
    [SerializeField] private float timer;
    [SerializeField] private float maxTime = 200;
    [SerializeField] private float meanScore = 0;
    [SerializeField] private double averageScore = 0;
    [SerializeField] private int generation = 0;

    [SerializeField] private GameObject geneticAgent;
    private List<GameObject> cars = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        timer = 0;

        SetSpot();

        SetAgent();
    }

    void SetSpot()
    {
        int spotSide = 1;// Random.Range(0, 2);
        int spotPosition = 3;// Random.Range(0, 7);

        float spotX = spotSide == 0 ? -9.2f : 9.2f;
        float spotZ = 16 - (spotPosition * 3.28f);
        float spotRotation = spotSide == 0 ? 180 : 0;

        target.localPosition = new Vector3(spotX, 0.05f, spotZ);
        target.localRotation = Quaternion.Euler(0, spotRotation, 270);
    }

    // Update is called once per frame
    void Update()
    {
        timer++;

        if (timer >= maxTime)
        {
            generation++;

            timer = 0;

            SetSpot();

            GameObject.Destroy(geneticAgent);

            SetAgent();
        }
    }*/

    void SetAgent()
    {/*
        string path = "NeuralNetworkImport/AgentNeuralNetwork.csv";
        geneticAgent = Instantiate(agent);
        DNA dna = geneticAgent.AddComponent<DNA>();
        dna.ResetDNA();
        dna.SetTarget(target);
        dna.SetNeuralNetwork(NeuralNetwork.LoadFromCSV(path));
        geneticAgent.transform.SetParent(environmentLocation);

        int randPosX = Random.Range(-4, 4); // Posição X aleatória
        float randRotY = Random.Range(-30, 31); // Rotação Y aleatória

        Vector3 agentLocation = new Vector3(randPosX, 0.3f, -16f);
        Quaternion agentRotation = Quaternion.Euler(0, randRotY, 0);

        geneticAgent.transform.localPosition = agentLocation;
        geneticAgent.transform.rotation = agentRotation;*/
    }
}
