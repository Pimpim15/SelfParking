using Assets.Scripts;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

public class CarAgent : MonoBehaviour
{
    /* ===================== parâmetros gerais ===================== */
    public NeuralNetwork Net;           // atribuída pelo GAController
    public GameObject Target;            // vaga de estacionamento
    public LayerMask ObstacleMask;      // defina “Obstacle” no Inspector
    private Renderer agentRenderer;
    private BoxCollider agentCollider;

    /* controle cinemático */
    const float MAX_STEER = 0.5f;      // rad
    const float MAX_SPEED = 6f;        // m/s
    const float ACCEL = 10f;           // m/s²
    const float WHEEL_BASE = 2f;       // distância entre eixos

    /* sensores */
    const int NUM_RAYS = 9;
    const float SENSOR_RANGE = 12f;

    /* episódio */
    const float MAX_TIME = 15f;        // s

    /* estado interno */
    public bool Done { get; private set; }
    public float Fitness { get; private set; }

    float speed, heading;
    float currentSteer = 0f;

    const float DRAG = 1.5f;   // s^-1 (ajuste)
    const float BRAKE = 15f;   // m/s² (ajuste)
    const float STEER_RATE = 1.8f; // rad/s - dinâmica de direção

    float timeAlive;
    bool collided = false;
    bool parked = false;
    bool isParking = false; // se o carro está tentando estacionar
    bool wasParking = false; // para detectar saída
    bool neverWasParking = true; // para evitar penalidade inicial
    Rigidbody rb;

    BoxCollider spotBox;

    // ====== Identidade & Logging ======
    public int Generation { get; private set; }
    public int AgentId   { get; private set; }

    [Serializable]
    public class AgentMetrics
    {
        public int generation;
        public int agentId;
        public bool parked;
        public bool collided;
        public float timeAlive;
        public float distance;
        public float maxSpeed;
        public float meanSpeed;
        public float meanAbsSteer;
        public float meanAbsThrottle;
        public int   gateEntries;
        public float insideGateTime;
        public float meanLateralError;
        public float meanHeadingAlign;   // cos(erro_ângulo)
        public float meanDirAlign;       // dot(v, direção_desejada)
        public float fitness;
        public int steps;
    }

    [Serializable]
    public class StepSample
    {
        public float t;
        public Vector3 pos;
        public float heading;     // rad
        public float speed;
        public float throttle;
        public float steer;       // comando (rad)
        public double[] obs;      // 19 entradas
        public bool insideGate;
        public bool parked;
        public float fitnessSoFar;
    }

    public AgentMetrics Metrics { get; private set; } = new AgentMetrics();

    // Acumuladores internos para médias
    float distSum, speedSum, steerAbsSum, throttleAbsSum, lateralSum, headingAlignSum, dirAlignSum, insideTimeSum;
    float maxSpeedAbs;
    int steps;

    // Trajetória (para análise de controle)
    List<StepSample> trajectory;
    bool recordTrajectory = false;
    int  maxTrajectorySteps = 600;
    const float LOG_DT = 0.1f;
    float logTimer = 0f;

    // --- descrição da vaga no mundo, independente da rotação do prefab
    struct SlotGeom
    {
        public Vector3 center;   // centro da BoxCollider (mundo)
        public Vector3 longDir;  // direção "para dentro" da vaga (mundo, unit)
        public Vector3 latDir;   // direção lateral (mundo, unit)
        public float halfLen;    // metade do comprimento
        public float halfWid;    // metade da largura
        public Vector3 entry;    // ponto de entrada (um pouco fora da "boca")
    }

    // Calcula eixos/entrada com base no BoxCollider e na posição do agente
    SlotGeom GetSlotGeometry(Vector3 agentPos, bool debug = false)
    {
        if(debug) Debug.Log($"Spot size: {spotBox.size}");

        var b = spotBox != null ? spotBox : Target.GetComponent<BoxCollider>();

        if(debug) Debug.Log($"Is b null? {b == null}");

        // fallback se não houver BoxCollider
        if (b == null)
        {
            return new SlotGeom
            {
                center = Target.transform.position,
                longDir = Target.transform.forward.normalized,
                latDir = Target.transform.right.normalized,
                halfLen = 2f,
                halfWid = 1f,
                entry = Target.transform.position + Target.transform.forward * 2f
            };
        }

        if(debug) Debug.Log($"Spot center: {b.center}, size: {b.size}");

        var t = b.transform;

        var sg = new SlotGeom();
        sg.center = b.transform.position;
        sg.longDir = Vector3.right;
        sg.latDir = Vector3.forward;
        sg.halfLen = 0.5f * b.size.y;
        sg.halfWid = 0.5f * b.size.z;

        var mouthA = sg.center - sg.longDir * sg.halfLen;
        var mouthB = sg.center + sg.longDir * sg.halfLen;
        var mouth = (Vector3.Distance(agentPos, mouthA) <= Vector3.Distance(agentPos, mouthB)) ? mouthA : mouthB;

        float clearance = 1.2f;
        sg.entry = mouth - sg.longDir * clearance;

        return sg;
    }

    struct AgentGeom
    {
        public Vector3 center;
        public Vector3 forward;
        public Vector3 right;
        public float halfWidth;
        public float halfLength;
    }

    AgentGeom GetAgentGeometry()
    {
        // usa o BoxCollider real (pode ser child) para obter centro/axes corretos em mundo
        if (agentCollider != null)
        {
            var colTransform = agentCollider.transform;
            Vector3 centerWorld = colTransform.TransformPoint(agentCollider.center);
            Vector3 forwardWorld = colTransform.forward;
            Vector3 rightWorld = colTransform.right;
            Vector3 s = colTransform.lossyScale;
            float halfWidth = (agentCollider.size.x * s.x) * 0.5f;
            float halfLength = (agentCollider.size.z * s.z) * 0.5f;

            return new AgentGeom
            {
                center = centerWorld,
                forward = forwardWorld,
                right = rightWorld,
                halfWidth = halfWidth,
                halfLength = halfLength
            };
        }

        // fallback para transform.root caso não haja collider
        return new AgentGeom
        {
            center = transform.position,
            forward = transform.forward,
            right = transform.right,
            halfWidth = 0.5f,
            halfLength = 1f
        };
    }

    /* ============================================================= */

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        agentRenderer = GetComponentInChildren<Renderer>();
        agentCollider = GetComponentInChildren<BoxCollider>();
    }

    // Novo Init com identidade + opção de registrar trajetória
    public void Init(NeuralNetwork net, GameObject tgt, LayerMask mask, int generation, int agentId, bool recordTraj = false, int trajMaxSteps = 600)
    {
        Net = net;
        Target = tgt;
        ObstacleMask = mask;

        Generation = generation;
        AgentId = agentId;

        recordTrajectory = recordTraj;
        maxTrajectorySteps = Mathf.Max(1, trajMaxSteps);
        trajectory = recordTrajectory ? new List<StepSample>(Mathf.Min(maxTrajectorySteps, 1024)) : null;

        spotBox = Target.GetComponent<BoxCollider>();
        //Debug.Log(Target);
        //Debug.Log(spotBox.size);

        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        speed = 0f;
        timeAlive = 0f;
        collided = false;
        parked = false;
        Done = false;
        Fitness = 0f;

        distSum = speedSum = steerAbsSum = throttleAbsSum = lateralSum = headingAlignSum = dirAlignSum = insideTimeSum = 0f;
        maxSpeedAbs = 0f;
        steps = 0;
        logTimer = 0f;

        Metrics = new AgentMetrics
        {
            generation = Generation,
            agentId = AgentId
        };
    }

    public event Action<CarAgent> EpisodeEnded;

    public void EnableTrajectoryLog(bool enable, int trajMaxSteps = 600)
    {
        recordTrajectory = enable;
        maxTrajectorySteps = Mathf.Max(1, trajMaxSteps);
        if (enable && trajectory == null) trajectory = new List<StepSample>(maxTrajectorySteps);
    }

    void FixedUpdate()
    {
        if (Done) return;

        timeAlive += Time.fixedDeltaTime * Time.timeScale; // escala de tempo do GAController

        Vector3 prevPos = transform.position;
        float dt = Time.fixedDeltaTime * Time.timeScale; // escala de tempo do GAController

        /* ---------- ação ---------- */
        double[] obs = BuildObservation();
        double[] act = Net.FeedForward(obs);

        float throttle = Mathf.Clamp((float)act[0], -1f, 1f);
        float steerCmd = Mathf.Clamp((float)act[1], -1f, 1f) * MAX_STEER;

        // Atualização de velocidade com arrasto e freagem
        speed += throttle * ACCEL * dt;
        speed -= DRAG * speed * dt;

        if (Mathf.Sign(speed) * Mathf.Sign(throttle) < 0f)
            speed += -Mathf.Sign(speed) * BRAKE * Mathf.Abs(throttle) * dt;

        if (Mathf.Abs(speed) < 0.05f) speed = 0f;
        speed = Mathf.Clamp(speed, -MAX_SPEED, MAX_SPEED);

        currentSteer = Mathf.MoveTowards(currentSteer, steerCmd, STEER_RATE * dt);

        float beta = Mathf.Atan(0.5f * Mathf.Tan(currentSteer));
        heading += (speed / WHEEL_BASE) * Mathf.Sin(2f * beta) * dt;

        Vector3 dir = new Vector3(Mathf.Sin(heading), 0, Mathf.Cos(heading));
        Vector3 newPos = prevPos + dir * speed * dt;
        Quaternion newRot = Quaternion.Euler(0, heading * Mathf.Rad2Deg, 0);

        rb.MovePosition(newPos);
        rb.MoveRotation(newRot);

        VerifyParked();

        /* ---------- métricas instantâneas ---------- */
        float stepDist = (newPos - prevPos).magnitude;
        distSum += stepDist;
        speedSum += Mathf.Abs(speed);
        steerAbsSum += Mathf.Clamp01(Mathf.Abs(currentSteer) / MAX_STEER);
        throttleAbsSum += Mathf.Abs(throttle);
        maxSpeedAbs = Mathf.Max(maxSpeedAbs, Mathf.Abs(speed));
        steps++;

        /* ---------- fitness & término ---------- */
        UpdateFitness(prevPos, newPos, dt, out bool wasInside, out bool isInside,
                      out float lateralErr, out float headingPose, out float dirAlign);

        // acumular para médias
        lateralSum += lateralErr;
        headingAlignSum += headingPose;
        dirAlignSum += dirAlign;
        if (isInside) insideTimeSum += dt;

        // logging de trajetória (10 Hz)
        if (recordTrajectory && (trajectory.Count < maxTrajectorySteps))
        {
            logTimer += dt;
            if (logTimer >= LOG_DT)
            {
                logTimer = 0f;
                trajectory.Add(new StepSample
                {
                    t = timeAlive,
                    pos = newPos,
                    heading = heading,
                    speed = speed,
                    throttle = throttle,
                    steer = currentSteer,
                    obs = (double[])obs.Clone(),
                    insideGate = isInside,
                    parked = parked,
                    fitnessSoFar = Fitness
                });
            }
        }

        if (TimeExceeded() || HasCollided() || IsParked())
        {
            if (TimeExceeded() && !parked) Fitness -= 10f;

            if (collided)
            {
                Fitness -= 5f;
                if (agentRenderer) agentRenderer.material.color = Color.red;
            }

            if (parked)
            {
                Fitness += 400f;
            }

            Done = true;
            FinalizeMetrics();
            EpisodeEnded?.Invoke(this);
        }
    }

    void FinalizeMetrics()
    {
        Metrics.parked = parked;
        Metrics.collided = collided;
        Metrics.timeAlive = timeAlive;
        Metrics.distance = distSum;
        Metrics.maxSpeed = maxSpeedAbs;
        Metrics.meanSpeed = steps > 0 ? speedSum / steps : 0f;
        Metrics.meanAbsSteer = steps > 0 ? steerAbsSum / steps : 0f;
        Metrics.meanAbsThrottle = steps > 0 ? throttleAbsSum / steps : 0f;
        Metrics.insideGateTime = insideTimeSum;
        Metrics.meanLateralError = steps > 0 ? lateralSum / steps : 0f;
        Metrics.meanHeadingAlign = steps > 0 ? headingAlignSum / steps : 0f;
        Metrics.meanDirAlign = steps > 0 ? dirAlignSum / steps : 0f;
        Metrics.fitness = Fitness;
        Metrics.steps = steps;
    }

    public AgentMetrics GetFinalMetrics() => Metrics;

    public List<StepSample> GetTrajectory() => trajectory;

    void UpdateFitness(Vector3 prevPos, Vector3 newPos, float dt,
                       out bool wasInside, out bool isInside,
                       out float lateralError, out float headingPose, out float dirAlign)
    {
        Vector3 entry = EntryPoint();
        Vector3 center = SlotCenter();
        var gNow = GetSlotGeometry(newPos);

        wasInside = IsPastGate(prevPos);
        isInside = IsPastGate(newPos);

        // entradas na vaga
        if (!wasInside && isInside)
        {
            Fitness += 10f;
            Metrics.gateEntries += 1;
            neverWasParking = false; // carro tentou estacionar
        }
        else if (wasInside && !isInside)
        {
            Fitness -= 8f; // penalidade por sair da vaga
        }
        else if (wasInside && isInside)
        {
            Fitness += 0.002f; // bônus por continuar dentro da vaga
        }
        else if (!isInside && !neverWasParking)
        {
            Fitness -= 0.001f; // penalidade por não ter tentado estacionar
        }

        // 1) progresso: até a boca (antes), depois até o centro
        float prevDist = Vector3.Distance(prevPos, wasInside ? center : entry);
        float currDist = Vector3.Distance(newPos, isInside ? center : entry);
        float progress = prevDist - currDist;            // aproximar = positivo

        // 2) alinhamento de heading com a vaga (pose)
        headingPose = HeadingAlign();

        // 3) erro lateral na faixa da vaga (quanto mais centrado, melhor)
        lateralError = LateralError(newPos);
        float laneHalf = gNow.halfWid + 0.5f;
        float lateralTerm = -Mathf.Clamp01(lateralError / laneHalf);

        // 4) seguir na direção “certa”
        Vector3 desired = ((isInside ? center : entry) - prevPos).normalized;
        Vector3 v = (newPos - prevPos);
        dirAlign = v.sqrMagnitude > 1e-6f ? Vector3.Dot(v.normalized, desired) : 0f;

        // 5) custo de tempo
        float timeCost = -0.0025f;

        // 6) ficar devagar perto do centro
        float nearCenter = Vector3.Distance(newPos, center);
        float slowDown = nearCenter < 1.5f ? -0.05f * Mathf.Abs(speed) : 0f;

        float insideLiving = isInside ? 0.01f : 0f;

        float r =
            1.00f * progress +
            0.30f * headingPose +
            0.20f * dirAlign +
            0.20f * lateralTerm +
            timeCost +
            slowDown +
            insideLiving;

        Fitness += r;

        float steerMag = Mathf.Clamp01(Mathf.Abs(currentSteer) / MAX_STEER);
        float speedMag = Mathf.Clamp01(Mathf.Abs(speed) / MAX_SPEED);
        float turningWhileMoving = (!isInside) ? -0.02f * steerMag * speedMag : 0f;

        Fitness += turningWhileMoving;

        float far = Vector3.Distance(newPos, gNow.center);
        if (!isInside && far > gNow.halfLen + 6f)
            Fitness -= 0.02f; // pune orbitar longe da boca
    }

    /* ================= helpers ================= */

    Vector3 SlotCenter() => GetSlotGeometry(transform.position).center;

    Vector3 EntryPoint() => GetSlotGeometry(transform.position).entry;

    bool IsPastGate(Vector3 p)
    {
        var g = GetSlotGeometry(p);
        // "dentro" = projetar além do plano da boca, no sentido do longDir
        return Vector3.Dot(p - g.entry, g.longDir) > 0f;
    }

    float HeadingAlign()
    {
        var g = GetSlotGeometry(transform.position);
        float ang = Mathf.Abs(Vector3.Dot(transform.forward, Vector3.right));
        return ang; // 1 quando alinhado com a direção esquerda - direita
    }

    float LateralError(Vector3 p)
    {
        var g = GetSlotGeometry(p);
        // desvio lateral absoluto em metros
        return Mathf.Abs(Vector3.Dot(p - g.center, g.latDir));
    }

    double[] BuildObservation()
    {
        List<double> obs = new List<double>();

        // 1) 9 raycasts
        for (int i = 0; i < NUM_RAYS; i++)
        {
            float angle = -90f + i * 22.5f;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, SENSOR_RANGE, ObstacleMask))
                obs.Add(1 - hit.distance / SENSOR_RANGE);
            else
                obs.Add(0);
        }

        // 2) Geometria real da vaga (robusta a rotação)
        var g = GetSlotGeometry(transform.position);

        // (a) Distância e bearing até a boca
        Vector3 toEntry = g.entry - transform.position;
        obs.Add(Mathf.Clamp01(toEntry.magnitude / 20f)); // distEntryNorm
        obs.Add(Vector3.SignedAngle(transform.forward, toEntry.normalized, Vector3.up) / 180f); // bearingEntry

        // (b) Offsets lateral/longitudinal no frame da vaga
        Vector3 delta = transform.position - g.center;
        float lat = Vector3.Dot(delta, g.latDir);
        float lon = Vector3.Dot(delta, g.longDir);
        obs.Add(Mathf.Clamp(lat / (g.halfWid + 0.5f), -1f, 1f)); // localXNorm
        obs.Add(Mathf.Clamp(lon / (g.halfLen + 1.0f), -1f, 1f));  // localZNorm

        // (c) Erro de heading para o eixo longitudinal real
        obs.Add(Vector3.SignedAngle(transform.forward, g.longDir, Vector3.up) / 180f);

        // (d) Velocidade normalizada
        obs.Add(Mathf.Clamp01(Mathf.Abs(speed) / MAX_SPEED));

        // (e) Orientação global
        obs.Add(Mathf.Sin(heading));
        obs.Add(Mathf.Cos(heading));

        // (f) Centro da vaga (dist e bearing)
        Vector3 toCenter = g.center - transform.position;
        obs.Add(Mathf.Clamp01(toCenter.magnitude / 20f));                      // distCenterNorm
        obs.Add(Vector3.SignedAngle(transform.forward, toCenter.normalized,    // bearingCenter
                Vector3.up) / 180f);

        return obs.ToArray(); // 19 entradas
    }

    /* ---------- condições de término ---------- */
    bool TimeExceeded() => timeAlive > MAX_TIME;
    bool HasCollided() => collided;
    bool IsParked() => parked;

    void VerifyParked()
    {
        var gNow = GetSlotGeometry(transform.position, false);
        var aNow = GetAgentGeometry();

        //Debug.Log($"gNow center: {gNow.center}, halfLen: {gNow.halfLen}, halfWid: {gNow.halfWid}");

        // calcular os 4 cantos usando os eixos do collider em mundo
        Vector3[] carCorners = new Vector3[4];
        carCorners[0] = aNow.center + ( - aNow.right * aNow.halfWidth) + ( - aNow.forward * aNow.halfLength);
        carCorners[1] = aNow.center + (   aNow.right * aNow.halfWidth) + ( - aNow.forward * aNow.halfLength);
        carCorners[2] = aNow.center + ( - aNow.right * aNow.halfWidth) + (   aNow.forward * aNow.halfLength);
        carCorners[3] = aNow.center + (   aNow.right * aNow.halfWidth) + (   aNow.forward * aNow.halfLength);

        // margem pequena para tolerância numérica / pintura ligeiramente fora
        const float margin = 0.08f;
        //Debug.Log($"Car corners: {string.Join(", ", carCorners.Select(c => c.ToString("F2")))}");
        //Debug.Log($"Slot center: {gNow.center.ToString("F2")}, halfLen: {gNow.halfLen:F2}, halfWid: {gNow.halfWid:F2}");

        bool AllCornersInsideSlot()
        {
            foreach (var corner in carCorners)
            {
                Vector3 rel = corner - gNow.center;
                float lat = Vector3.Dot(rel, gNow.latDir);
                float lon = Vector3.Dot(rel, gNow.longDir);
                //Debug.Log($"Corner at lat {lat:F2} m, lon {lon:F2} m");
                if (Mathf.Abs(lat) > gNow.halfWid + margin || Mathf.Abs(lon) > gNow.halfLen + margin)
                    return false;
            }
            return true;
        }

        if (AllCornersInsideSlot())
        {
            parked = true;
            if (agentRenderer) agentRenderer.material.color = Color.green;
            return;
        }
    }

    /* ---------- detecção de colisão ---------- */
    void OnCollisionEnter(Collision col)
    {
        if (col.collider.CompareTag("Spot"))
            return;

        int L(Collider c) => c.transform.root.gameObject.layer;
        if (((1 << col.collider.gameObject.layer) & ObstacleMask) != 0 ||
            ((1 << L(col.collider)) & ObstacleMask) != 0)
            collided = true;
    }

    void OnTriggerEnter(Collider other)
    {        
        if (other.CompareTag("Spot") && other is BoxCollider)
        {
            isParking = true;
            wasParking = false;
            neverWasParking = false; // carro tentou estacionar
        }
    }
    
    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Spot") && other is BoxCollider)
        {
            isParking = false;
            wasParking = true; 
        }
    }
}
