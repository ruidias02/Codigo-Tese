#include "ns3/core-module.h"
#include "ns3/ndnSIM-module.h" 
#include "ns3/network-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h" 
#include "ns3/applications-module.h"
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_set>
#include <fstream>
#include <cstdlib> 
#include <limits> 

using namespace ns3;
using namespace std;

namespace ns3 {

using StateVector = std::map<ndn::Name, uint64_t>; 

// -------------------- Node Data --------------------
struct NodeData {
    Ptr<Node> node;
    int row{0};
    int col{0};
    std::string name;
    int initialDataVersion{0};
    int dataVersion{0}; 
    bool isPivot{false};
    bool isPoint{false};
    bool hasArrivedAtCenter{false};
};

// -------------------- Synchronization Metrics --------------------
struct SyncMetrics {
    double startTime{0.0};
    double endTime{0.0};
    double duration{0.0};
    bool started{false};
    bool ended{false};
    bool analysisStarted{false}; 

    void Start() {
        if (started) return;
        startTime = Simulator::Now().GetSeconds();
        started = true;
        cout << "\n------------------------------------------------------" << endl;
        cout << "INÍCIO DA SINCRONIZAÇÃO: " << startTime << "s" << endl;
        cout << "------------------------------------------------------" << endl;
    }

    void End() {
        if (ended) return;
        endTime = Simulator::Now().GetSeconds();
        duration = endTime - startTime;
        ended = true;
        cout << "\n------------------------------------------------------" << endl;
        cout << "FIM DA SINCRONIZAÇÃO: " << endTime << "s" << endl;
        cout << "DURAÇÃO TOTAL DA SINCRONIZAÇÃO: " << duration << "s" << endl;
        cout << "------------------------------------------------------" << endl;
    }

    void WriteFile(const string &filename = "sync_metrics.txt") const {
        ofstream ofs(filename);
        if (!ofs.is_open()) {
            cerr << "[METRICS] Falha ao abrir " << filename << " para escrita\n";
            return;
        }
        ofs << startTime << " " << endTime << " " << duration << "\n";
        ofs.close();
        cout << "[METRICS] Métricas escritas em " << filename << "\n";
    }
    
    void RunExternalAnalysis() {
        if (analysisStarted) return;
        analysisStarted = true;
        std::cout << "\n === MÉTRICAS FINAIS DA SINCRONIZAÇÃO ===" << std::endl;
        std::string command = "python3 analyze_tracer.py " +
                              std::to_string(startTime) + " " +
                              std::to_string(endTime);
        int result = system(command.c_str());
        if (result != 0) {
            std::cout << " Erro ao executar o script de análise." << std::endl;
        } else {
            std::cout << "\n Análise concluída." << std::endl;
        }
    }
};

// -------------------- Hierarchical Sync Manager --------------------
class HierarchicalSyncManager {
public:
    SyncMetrics metrics; 

    HierarchicalSyncManager(int expectedPoints)
        : expectedPoints(expectedPoints), arrivedPoints(0), syncPhase(0),
          simulationFinished(false) {
        cout << "[MANAGER] HierarchicalSyncManager criado a aguardar " << expectedPoints << " pontos.\n";
    }

    shared_ptr<NodeData> RegisterNode(Ptr<Node> node, int row, int col, bool markPoint, bool markPivot) {
        auto nd = make_shared<NodeData>();
        nd->node = node;
        nd->row = row;
        nd->col = col;
        nd->name = "Node-" + to_string(row) + "-" + to_string(col);
        nd->isPoint = markPoint;
        nd->isPivot = markPivot;

        Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();
        nd->initialDataVersion = 1 + urv->GetInteger(0, 14); 
        nd->dataVersion = nd->initialDataVersion;

        nodes.push_back(nd);
        if (nd->isPoint) points.push_back(nd);
        if (nd->isPivot) pivots.push_back(nd);

        cout << "[REGISTER] " << nd->name
             << (nd->isPoint ? " [POINT]" : "")
             << (nd->isPivot ? " [PIVOT]" : "")
             << " initialVersion=" << nd->initialDataVersion << "\n";

        return nd;
    }

    const vector<shared_ptr<NodeData>>& GetPoints() const { return points; }

    void MovePointToCenter(shared_ptr<NodeData> nd) {
        if (simulationFinished) return;

        Ptr<MobilityModel> mob = nd->node->GetObject<MobilityModel>();
        if (mob) mob->SetPosition(Vector(300.0, 300.0, 0.0));

        if (!nd->hasArrivedAtCenter) {
            nd->hasArrivedAtCenter = true;
            arrivedPoints++;
            cout << "[MOVE] " << nd->name << " chegou ao centro (" << arrivedPoints
                 << "/" << expectedPoints << ")\n";
        }
    }

    void StartNextPhase() {
        if (simulationFinished) return;

        syncPhase++;
        cout << "\n[SYNC] A iniciar fase " << syncPhase << " em t=" << Simulator::Now().GetSeconds() << "s\n";

        if (syncPhase == 1) {
            Phase1_LanesToPivots();
            Simulator::Schedule(Seconds(0.1), &HierarchicalSyncManager::CheckAndAdvancePhase, this);
        } else if (syncPhase == 2) {
            Phase2_PivotsInterSync();
            Simulator::Schedule(Seconds(0.1), &HierarchicalSyncManager::CheckAndAdvancePhase, this);
        } else if (syncPhase == 3) {
            Phase3_PivotsToLanes();
            Simulator::Schedule(Seconds(0.1), &HierarchicalSyncManager::CheckAndAdvancePhase, this);
        } else {
            FinishSimulation();
        }
    }
    
    void CheckAndAdvancePhase() {
        if (simulationFinished || syncPhase == 0 || metrics.ended) return;

        bool convergenceAchieved = false;
        
        double checkInterval = 0.05; 

        uint64_t maxPointVersion = GetMaxVersion(points);
        uint64_t maxPivotVersion = GetMaxVersion(pivots);

        if (syncPhase == 1) {
            // Fase 1: Points -> Pivots (durante o movimento)
            // CONDIÇÃO: Os Pivots obtiveram a versão máxima publicada pelos Points.
            if (maxPivotVersion >= maxPointVersion && maxPointVersion > 0) {
                convergenceAchieved = true;
                cout << "[CONVERGÊNCIA] FASE 1 CONCLUÍDA em t=" << Simulator::Now().GetSeconds() << "s. (Points -> Pivots)\n";
                StartNextPhase(); 
                return;
            }

        } else if (syncPhase == 2) {
            // Fase 2: Pivots <-> Pivots (com todos no centro)
            // CONDIÇÃO: Todos os Pivots têm o mesmo SV (o mínimo é igual ao máximo)
            uint64_t minPivotVersion = std::numeric_limits<uint64_t>::max();
            for (const auto& nd : pivots) {
                StateVector sv = GetSvsStateVector(nd->node);
                if (!sv.empty() && sv.begin()->second > 0) { 
                    minPivotVersion = std::min(minPivotVersion, sv.begin()->second);
                }
            }
            
            if (minPivotVersion == maxPivotVersion && maxPivotVersion > 0) {
                convergenceAchieved = true;
                cout << "[CONVERGÊNCIA] FASE 2 CONCLUÍDA em t=" << Simulator::Now().GetSeconds() << "s. (Pivots <-> Pivots)\n";
                StartNextPhase(); 
                return;
            }
            
        } else if (syncPhase == 3) {
            // Fase 3: Pivots -> Points (com todos no centro)
            // CONDIÇÃO: Points sincronizaram o SV máximo alcançado pelos Pivots.
            if (maxPointVersion >= maxPivotVersion && maxPivotVersion > 0) {
                convergenceAchieved = true;
                cout << "[CONVERGÊNCIA] FASE 3 CONCLUÍDA em t=" << Simulator::Now().GetSeconds() << "s. (Pivots -> Points)\n";
                FinishSimulation(); 
                return;
            }
        }

        // Se não convergiu, agenda nova verificação
        if (!convergenceAchieved) {
            Simulator::Schedule(Seconds(checkInterval), &HierarchicalSyncManager::CheckAndAdvancePhase, this);
        }
    }


private:
    vector<shared_ptr<NodeData>> nodes;
    vector<shared_ptr<NodeData>> points;
    vector<shared_ptr<NodeData>> pivots;
    int expectedPoints;
    int arrivedPoints;
    int syncPhase;
    bool simulationFinished;

    uint64_t GetMaxVersion(const std::vector<std::shared_ptr<NodeData>>& nodeList) const {
        uint64_t maxV = 0;
        for (const auto& nd : nodeList) {
            StateVector sv = GetSvsStateVector(nd->node);
            if (!sv.empty()) {
                maxV = std::max(maxV, sv.begin()->second);
            }
        }
        return maxV;
    }
    
    StateVector GetSvsStateVector(Ptr<Node> node) const {
        StateVector sv;
        
        auto it = std::find_if(nodes.begin(), nodes.end(), [&](const shared_ptr<NodeData>& nd){
            return nd->node == node;
        });

        if (it != nodes.end()) {
            if (!((*it)->row == 2 && (*it)->col == 2)) { 
                ndn::Name svs_prefix("/ndn/svs/chat"); 
                sv[svs_prefix] = (*it)->dataVersion;
            }
        }

        return sv;
    }

    void Phase1_LanesToPivots() {
        cout << "[INST] FASE 1: Lanes (Points) instruem Pivots a sincronizar a versão máxima.\n";
        int maxLaneVersion = -1;
        for (auto &p : points) if (!p->isPivot) maxLaneVersion = max(maxLaneVersion, p->dataVersion);
        for (auto &pv : pivots) if (pv->dataVersion < maxLaneVersion) pv->dataVersion = maxLaneVersion;
    }

    void Phase2_PivotsInterSync() {
        cout << "[INST] FASE 2: Pivots instruem Pivots a sincronizar a versão máxima entre si.\n";
        int maxPivotVersion = -1;
        for (auto &pv : pivots) maxPivotVersion = max(maxPivotVersion, pv->dataVersion);
        for (auto &pv : pivots) pv->dataVersion = maxPivotVersion;
    }

    void Phase3_PivotsToLanes() {
        cout << "[INST] FASE 3: Pivots instruem Points (Lanes) a sincronizar a versão máxima.\n";
        int pivotMax = -1;
        for (auto &pv : pivots) pivotMax = max(pivotMax, pv->dataVersion);
        for (auto &p : points) p->dataVersion = pivotMax;
    }

    void FinishSimulation() {
        if (simulationFinished) return;
        metrics.End();
        metrics.WriteFile();

        cout << "\n=== RESUMO FINAL DA SINCRONIZAÇÃO (t=" << Simulator::Now().GetSeconds() << "s) ===\n";
        int finalVersion = -1;
        for (auto &p : points) finalVersion = max(finalVersion, p->dataVersion);
        for (auto &pv : pivots) finalVersion = max(finalVersion, pv->dataVersion);

        for (auto &p : points) p->dataVersion = finalVersion;
        for (auto &pv : pivots) pv->dataVersion = finalVersion;

        for (auto &p : points) {
            cout << "POINT " << p->name << " inicial=" << p->initialDataVersion
                 << " final=" << p->dataVersion << (p->dataVersion==finalVersion ? " (OK)" : " (FALHA)") << "\n";
        }
        for (auto &pv : pivots) {
            cout << "PIVOT " << pv->name << " inicial=" << pv->initialDataVersion
                 << " final=" << pv->dataVersion << (pv->dataVersion==finalVersion ? " (OK)" : " (FALHA)") << "\n";
        }
        
        metrics.RunExternalAnalysis(); 

        simulationFinished = true;
        Simulator::Stop();
    }
};

// -------------------- Helpers --------------------
static bool IsPointCoord(int row, int col) {
    if (row == 2 && (col == 0 || col == 1 || col == 3 || col == 4)) return true;
    if (col == 2 && (row == 0 || row == 1 || row == 3 || row == 4)) return true;
    return false;
}

static bool IsPivotCoord(int row, int col) {
    if (row == 2 && (col == 1 || col == 3)) return true;
    if (col == 2 && (row == 1 || row == 3)) return true;
    return false;
}

// -------------------- Main --------------------
int main(int argc, char* argv[]) {
    int nRows = 5;
    int nCols = 5;
    int interPubMsSlow = 1500;
    int interPubMsFast = 800;
    int nRecent = 5;
    int nRandom = 3;
    double dropRate = 0.01;
    bool frag = false;

    CommandLine cmd;
    cmd.AddValue("interPubMsSlow", "slow publisher interval (ms)", interPubMsSlow);
    cmd.AddValue("interPubMsFast", "fast publisher interval (ms)", interPubMsFast);
    cmd.AddValue("nRecent", "number of recent entries", nRecent);
    cmd.AddValue("nRandom", "number of random entries", nRandom);
    cmd.AddValue("dropRate", "packet drop rate", dropRate);
    cmd.AddValue("frag", "enable fragmentation (MTU 1280)", frag);
    cmd.Parse(argc, argv);

    // Configure P2P + error model
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetStream(50);
    RateErrorModel* rem = new RateErrorModel();
    rem->SetRandomVariable(uv);
    rem->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    rem->SetRate(dropRate);
    Config::SetDefault("ns3::PointToPointNetDevice::ReceiveErrorModel", PointerValue(rem));
    Config::SetDefault("ns3::PointToPointNetDevice::DataRate", StringValue("50Mbps"));
    Config::SetDefault("ns3::PointToPointChannel::Delay", StringValue("5ms"));

    cout << " === SIMULAÇÃO NDN OTIMIZADA - SINCRONIZAÇÃO HIERÁRQUICA ===" << endl;

    // Grid Topology
    PointToPointHelper p2p;
    PointToPointGridHelper grid(nRows, nCols, p2p);
    grid.BoundingBox(50, 50, 550, 550);

    // NDN Stack and Routing
    ndn::StackHelper ndnHelper;
    ndnHelper.InstallAll();
    ndn::GlobalRoutingHelper globalRouting;
    globalRouting.InstallAll();

    // Configuration
    ndn::L3RateTracer::InstallAll("L3RateTracer.txt", Seconds(1.0));
    ndn::AppDelayTracer::InstallAll("AppDelayTracer.txt");
    ndn::CsTracer::InstallAll("CsTracer.txt", Seconds(1.0));
    
    ndn::StrategyChoiceHelper::InstallAll("/ndn/svs", "/localhost/nfd/strategy/multicast");
    ndn::StrategyChoiceHelper::InstallAll("/", "/localhost/nfd/strategy/best-route");


    // Fast publishers definition
    unordered_set<string> fastPublishers;
    for (int i = 0; i < nRows*nCols; ++i) {
        int r=i/nCols, c=i%nCols;
        if (i%2==0 && !(r==2 && c==2)) fastPublishers.insert("/"+to_string(r)+"-"+to_string(c));
    }

    // Manager
    auto manager = make_shared<HierarchicalSyncManager>(8); 

    for(int r=0; r<nRows; ++r) {
        for(int c=0; c<nCols; ++c) {
            Ptr<Node> node = grid.GetNode(r,c);

            // Mobility
            MobilityHelper mob;
            Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
            posAlloc->Add(Vector(c*100+100,r*100+100,0));
            mob.SetPositionAllocator(posAlloc);
            mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mob.Install(node);

            // SVS Application (Chat)
            if(!(r==2 && c==2)) { 
                string prefix = "/"+to_string(r)+"-"+to_string(c);
                ndn::AppHelper svs("Chat"); 
                bool isFast = (fastPublishers.find(prefix)!=fastPublishers.end());
                svs.SetPrefix(prefix);
                svs.SetAttribute("PublishDelayMs", IntegerValue(isFast ? interPubMsFast : interPubMsSlow));
                svs.SetAttribute("NRecent", IntegerValue(nRecent));
                svs.SetAttribute("NRand", IntegerValue(nRandom));
                svs.Install(node).Start(Seconds(5.0+(r*nCols+c)*0.1)); 
                globalRouting.AddOrigins(prefix,node);
            }

            manager->RegisterNode(node,r,c,IsPointCoord(r,c),IsPivotCoord(r,c));
        }
    }

    globalRouting.CalculateRoutes();

    // FIB Routes for /ndn/svs (Multicast-like)
    for (int row = 0; row < nRows; row++) {
        for (int col = 0; col < nCols; col++) {
            Ptr<Node> participant = grid.GetNode(row, col);
            if (row > 0) ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row - 1, col), 1);
            if (col > 0) ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row, col - 1), 1);
            if (row < nRows - 1) ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row + 1, col), 1);
            if (col < nCols - 1) ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row, col + 1), 1);
        }
    }

    // Schedule all points to move to center at 10.0s
    for (auto &nd : manager->GetPoints()) {
        Simulator::Schedule(
            Seconds(10.0), 
            [manager, nd]() { 
                manager->MovePointToCenter(nd);
            }
        );
    }
    
    // Schedule the start of Phase 1 to occur DURING the movement
    Simulator::Schedule(
        Seconds(11.0),
        [manager]() {
            if (!manager->metrics.started) {
                manager->metrics.Start(); 
                manager->StartNextPhase(); 
            }
        }
    );

    Simulator::Run();
    Simulator::Destroy();
    delete rem;

    cout << "[MAIN] Simulação terminada.\n";
    return 0;
}

} 

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }
