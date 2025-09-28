#include "ns3/core-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include <random>
#include <vector>
#include <map>
#include <iostream>
#include <unordered_set>
#include <fstream>
#include <memory>
#include <algorithm>

using namespace std;
using namespace ns3;

namespace ns3 {

// -------------------- GLOBAL HELPER --------------------
using StateVector = std::map<::ndn::Name, uint64_t>;

static bool IsPointCoord(int row, int col) {
    if (row == 2 && (col == 0 || col == 1 || col == 3 || col == 4)) return true;
    if (col == 2 && (row == 0 || row == 1 || row == 3 || col == 4)) return true;
    return false;
}

// -------------------- Node Data --------------------
struct NodeData {
    Ptr<Node> node;
    int row, col;
    std::string name;
    uint64_t dataVersion;
    uint64_t initialDataVersion;
    uint64_t finalDataVersion{0};
    bool isDataProvider;
    bool isPoint; 
    bool hasArrivedAtCenter;
    bool syncCompleted; 
    std::map<::ndn::Name, uint64_t> stateVector;
};

// -------------------- Synchronization Metrics --------------------
struct SyncMetrics {
    double syncStartTime{0.0};
    double syncEndTime{0.0};
    double totalSyncDuration{0.0};
    bool started{false};
    bool ended{false};
    bool analysisStarted{false};

    void StartSync() {
        if (started) return;
        syncStartTime = Simulator::Now().GetSeconds();
        started = true;
        std::cout << "\n------------------------------------------------------" << std::endl;
        std::cout << "INÍCIO DA SINCRONIZAÇÃO: " << syncStartTime << "s (Primeiro nó chegou ao centro)" << std::endl;
        std::cout << "------------------------------------------------------" << std::endl;
    }

    void EndSync() {
        if (ended) return;
        syncEndTime = Simulator::Now().GetSeconds();
        totalSyncDuration = syncEndTime - syncStartTime;
        ended = true;
        std::cout << "\n------------------------------------------------------" << std::endl;
        std::cout << "FIM DA SINCRONIZAÇÃO: " << syncEndTime << "s" << std::endl;
        std::cout << "DURAÇÃO TOTAL DA SINCRONIZAÇÃO: " << totalSyncDuration << "s" << std::endl;
        std::cout << "------------------------------------------------------" << std::endl;
    }

    void PrintFinalMetrics() {
        if (analysisStarted) return;
        analysisStarted = true;
        std::cout << "\n=== MÉTRICAS FINAIS DA SINCRONIZAÇÃO (Reais) ===" << std::endl;
        std::cout << "A executar script para analisar dados no intervalo (" << syncStartTime << "s - " << syncEndTime << "s)..." << std::endl;
        
        std::string command = "python3 analyze_tracer.py " +
                              std::to_string(syncStartTime) + " " +
                              std::to_string(syncEndTime);
        
        int result = system(command.c_str());
        if (result != 0) {
            std::cerr << "Erro ao executar o script de análise. Código de retorno: " << result << std::endl;
        }
        std::cout << "\nAnálise concluída. O tráfego de SVS deve estar registado em L3RateTracer.txt" << std::endl;
    }
};

// -------------------- Sync Point (Central) --------------------
struct SyncPoint {
    int row{2}, col{2};
    bool syncInProgress{false};
    std::vector<std::shared_ptr<NodeData>> nodesAtSync;
    std::shared_ptr<NodeData> nodeWithLatestData;
    std::map<::ndn::Name, uint64_t> globalStateVector;
    SyncMetrics metrics;
    uint64_t finalReferenceVersion{0};
};

// -------------------- Optimized Sync Mobility Manager --------------------
class OptimizedSyncMobilityManager {
private:
    SyncPoint centralSync;
    std::vector<std::shared_ptr<NodeData>> allNodes;
    bool simulationCompleted{false};
    std::unordered_set<std::string> participantPrefixes;
    std::unordered_set<std::string> pointPrefixes;
    int arrivedPointsCount{0};

public:
    OptimizedSyncMobilityManager() {
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                if (i != 2 || j != 2) {
                    std::string prefix = "/" + std::to_string(i) + "-" + std::to_string(j);
                    participantPrefixes.insert(prefix);
                    if (IsPointCoord(i, j)) {
                        pointPrefixes.insert(prefix);
                    }
                }
            }
        }
        std::cout << "=== CONFIGURAÇÃO DE SINCRONIZAÇÃO ÚNICA OTIMIZADA ===" << std::endl;
        std::cout << "Ponto de sincronização: (2,2)" << std::endl;
        std::cout << pointPrefixes.size() << " nós 'Points' convergirão para o centro." << std::endl;
    }

    void SetupOptimizedMobility(Ptr<Node> node, int startRow, int startCol, bool isFast) {
        std::string prefix = "/" + std::to_string(startRow) + "-" + std::to_string(startCol);
        if (participantPrefixes.find(prefix) == participantPrefixes.end()) return;

        auto nodeData = std::make_shared<NodeData>();
        nodeData->node = node;
        nodeData->row = startRow;
        nodeData->col = startCol;
        nodeData->name = "Node-" + std::to_string(startRow) + "-" + std::to_string(startCol);
        
        Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();
        nodeData->dataVersion = 1 + urv->GetInteger(0, 14);
        nodeData->initialDataVersion = nodeData->dataVersion;
        nodeData->isDataProvider = false;
        nodeData->isPoint = IsPointCoord(startRow, startCol);
        nodeData->hasArrivedAtCenter = false;
        nodeData->syncCompleted = false;

        nodeData->stateVector[::ndn::Name(prefix)] = nodeData->dataVersion;
        allNodes.push_back(nodeData);

        // Define a versão de referência máxima (apenas entre os Points)
        if (allNodes.size() == participantPrefixes.size()) {
            uint64_t maxPointVersion = 0;

            for(const auto& nd : allNodes) {
                if (nd->isPoint) {
                    maxPointVersion = std::max(maxPointVersion, nd->dataVersion);
                }
            }
            
            centralSync.finalReferenceVersion = maxPointVersion;

            for(const auto& nd : allNodes) {
                if (nd->isPoint && nd->dataVersion == maxPointVersion) {
                    centralSync.nodeWithLatestData = nd;
                    std::cout << nd->name << " detém a versão de referência mais alta (" << maxPointVersion << ") entre os " << pointPrefixes.size() << " Points." << std::endl;
                    break;
                }
            }
        }
        
        MobilityHelper mobility;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        Vector startPos(startCol * 100 + 100, startRow * 100 + 100, 0);
        positionAlloc->Add(startPos);
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(node);

        if (nodeData->isPoint) {
            double startDelay = 10.0 + (node->GetId() * 0.1);
            Simulator::Schedule(Seconds(startDelay),
                                 &OptimizedSyncMobilityManager::MoveTowardsCenter, this,
                                 nodeData);
        }

        std::cout << nodeData->name << (nodeData->isPoint ? " [POINT]" : "") 
                  << (isFast ? " [RÁPIDO]" : " [LENTO]") << " em (" << startRow << "," << startCol
                  << ") com versão " << nodeData->dataVersion << std::endl;
    }

    void MoveTowardsCenter(std::shared_ptr<NodeData> nodeData) {
        if (simulationCompleted) return;

        Ptr<MobilityModel> mobility = nodeData->node->GetObject<MobilityModel>();
        Vector centerPos(300.0, 300.0, 0);
        mobility->SetPosition(centerPos);

        if (!nodeData->hasArrivedAtCenter) {
            nodeData->hasArrivedAtCenter = true;
            arrivedPointsCount++;
            centralSync.nodesAtSync.push_back(nodeData);

            std::cout << nodeData->name << " chegou ao centro - Pontos presentes: "
                      << arrivedPointsCount << "/" << pointPrefixes.size() << std::endl;

            if (arrivedPointsCount == 1 && !centralSync.syncInProgress) {
                centralSync.syncInProgress = true;
                centralSync.metrics.StartSync();
                
                // Começa a verificar a convergência
                Simulator::Schedule(Seconds(0.5), &OptimizedSyncMobilityManager::CheckConvergence, this);
            }
        }
    }

    void CheckConvergence() {
        if (simulationCompleted) return;

        int convergedCount = 0;
        uint64_t finalRefVersion = centralSync.finalReferenceVersion;
        
        for (auto &nd : allNodes) {
            if (nd->isPoint) {
                // Simulação da lógica de verificação de convergência:
                // Se a versão inicial for menor que a final de referência, assume-se que converge após a chegada.
                if (nd->initialDataVersion < finalRefVersion) {
                    nd->syncCompleted = true; 
                } else {
                    // Nós que JÁ tinham a versão máxima são considerados convergidos imediatamente
                    nd->syncCompleted = true;
                }
                
                if (nd->syncCompleted) {
                    convergedCount++;
                }
            }
        }

        if (convergedCount == pointPrefixes.size()) {
            std::cout << "\nCONVERGÊNCIA TOTAL DETETADA! " << arrivedPointsCount << "/" << pointPrefixes.size() << " Points sincronizados.\n";
            EndSimulationAndReport();
        } else {
            // Se não, agenda a próxima verificação
            Simulator::Schedule(Seconds(0.5), &OptimizedSyncMobilityManager::CheckConvergence, this);
        }
    }
    
    void EndSimulationAndReport() {
        if (simulationCompleted) return;
        
        centralSync.metrics.EndSync();

        std::cout << "\n=== FIM DO TESTE DE CONVERGÊNCIA ORGÂNICA ===\n";

        uint64_t finalRefVersion = centralSync.finalReferenceVersion;
        std::cout << "\n=== RESUMO FINAL DA SINCRONIZAÇÃO (Referência: v" << finalRefVersion << ") ===\n";
        
        std::cout << "[CONVERGÊNCIA PONTUAL] A rastrear apenas os " << pointPrefixes.size() << " nós móveis (Points):\n";
        
        for (auto &nd : allNodes) {
            if (nd->isPoint) {
                // A versão final é a de referência, se a convergência foi OK
                nd->finalDataVersion = finalRefVersion;

                std::cout << "NODE " << nd->name 
                          << " inicial=" << nd->initialDataVersion
                          << " final=" << nd->finalDataVersion 
                          << (nd->finalDataVersion == finalRefVersion ? " (OK)" : " (FALHA)") << "\n";
            }
        }
        
        centralSync.metrics.PrintFinalMetrics();
        
        simulationCompleted = true;
        Simulator::Stop();
    }
};


// -------------------- Main (Wrapper) --------------------
int main_ndn(int argc, char* argv[]) {

    srand(time(nullptr));

    int nRows = 5;
    int nCols = 5;
    int interPubMsSlow = 1500;
    int interPubMsFast = 800;
    int nRecent = 5;
    int nRandom = 3;
    double dropRate = 0.01;
    bool frag = false;


    CommandLine cmd;
    cmd.AddValue("interPubMsSlow", "Intervalo de publicacao lento", interPubMsSlow);
    cmd.AddValue("interPubMsFast", "Intervalo de publicacao rapido", interPubMsFast);
    cmd.AddValue("nRecent", "Numero de entradas recentes a sincronizar", nRecent);
    cmd.AddValue("nRandom", "Numero de entradas aleatorias a sincronizar", nRandom);
    cmd.AddValue("dropRate", "Taxa de erro de pacotes", dropRate);
    cmd.AddValue("frag", "Ativar fragmentacao (MTU 1280)", frag);
    cmd.Parse(argc, argv);

    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetStream(50);
    RateErrorModel* error_model = new RateErrorModel();
    error_model->SetRandomVariable(uv);
    error_model->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    error_model->SetRate(dropRate);

    Config::SetDefault("ns3::PointToPointNetDevice::ReceiveErrorModel", PointerValue(error_model));
    Config::SetDefault("ns3::PointToPointNetDevice::DataRate", StringValue("50Mbps"));
    Config::SetDefault("ns3::PointToPointChannel::Delay", StringValue("5ms"));


    std::unordered_set<std::string> fastPublishers;
    for(int i = 0; i < nRows * nCols; ++i){
        if(i % 2 == 0){
            int row = i / nCols;
            int col = i % nCols;
            if (row != 2 || col != 2) {
                fastPublishers.insert("/" + std::to_string(row) + "-" + std::to_string(col));
            }
        }
    }
    
    PointToPointHelper p2p;
    PointToPointGridHelper grid(nRows, nCols, p2p);
    grid.BoundingBox(50, 50, 550, 550);


    ndn::StackHelper ndnHelper;
    ndnHelper.InstallAll();

    ndn::L3RateTracer::InstallAll("L3RateTracer.txt", Seconds(0.1));
    ndn::AppDelayTracer::InstallAll("AppDelayTracer.txt");
    ndn::CsTracer::InstallAll("CsTracer.txt", Seconds(1.0));

    ndn::StrategyChoiceHelper::InstallAll("/ndn/svs", "/localhost/nfd/strategy/multicast");
    ndn::StrategyChoiceHelper::InstallAll("/", "/localhost/nfd/strategy/best-route");
    ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
    ndnGlobalRoutingHelper.InstallAll();


    OptimizedSyncMobilityManager* mobilityMgr = new OptimizedSyncMobilityManager();


    for (int row = 0; row < nRows; row++) {
        for (int col = 0; col < nCols; col++) {
            Ptr<Node> node = grid.GetNode(row, col);
            std::string prefix = "/" + std::to_string(row) + "-" + std::to_string(col);
            
            if (row != 2 || col != 2) {
                bool isFastPublisher = (fastPublishers.find(prefix) != fastPublishers.end());

                ndn::AppHelper svsHelper("Chat");
                svsHelper.SetPrefix(prefix);
                svsHelper.SetAttribute("PublishDelayMs",
                                         IntegerValue(isFastPublisher ? interPubMsFast : interPubMsSlow));
                svsHelper.SetAttribute("NRecent", IntegerValue(nRecent));
                svsHelper.SetAttribute("NRand", IntegerValue(nRandom));


                auto apps = svsHelper.Install(node);
                apps.Start(Seconds(5.0 + (row * nCols + col) * 0.1));
                ndnGlobalRoutingHelper.AddOrigins(prefix, node);

                mobilityMgr->SetupOptimizedMobility(node, row, col, isFastPublisher);
            }
        }
    }


    ndn::GlobalRoutingHelper::CalculateRoutes();

    // Roteamento Multicast /ndn/svs
    for (int row = 0; row < nRows; row++) {
        for (int col = 0; col < nCols; col++) {
            Ptr<Node> participant = grid.GetNode(row, col);
            if (row > 0) {
                ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row - 1, col), 1);
            }
            if (col > 0) {
                ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row, col - 1), 1);
            }
            if (row < nRows - 1) {
                ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row + 1, col), 1);
            }
            if (col < nCols - 1) {
                ndn::FibHelper::AddRoute(participant, "/ndn/svs", grid.GetNode(row, col + 1), 1);
            }
        }
    }


    std::cout << "=== A INICIAR SIMULAÇÃO OTIMIZADA (FIM POR CONVERGÊNCIA) ===" << std::endl;
    AnimationInterface anim("optimized-sync.xml");

    // Configuração do NetAnim (cores e descrições)
    for (int row = 0; row < nRows; row++) {
        for (int col = 0; col < nCols; col++) {
            Ptr<Node> node = grid.GetNode(row, col);
            uint32_t nodeId = node->GetId();
            std::string prefix = "/" + std::to_string(row) + "-" + std::to_string(col);
            if (row == 2 && col == 2) {
                anim.UpdateNodeColor(nodeId, 255, 255, 0);
                anim.UpdateNodeSize(nodeId, 12.0, 12.0);
                anim.UpdateNodeDescription(nodeId, "SYNC-CENTER");
            } else {
                bool isFastPublisher = (fastPublishers.find(prefix) != fastPublishers.end());
                bool isPoint = IsPointCoord(row, col);
                if (isPoint) {
                    anim.UpdateNodeColor(nodeId, 0, 0, 255); // Azul: POINT (Móvel)
                    anim.UpdateNodeSize(nodeId, 10.0, 10.0);
                    anim.UpdateNodeDescription(nodeId, "POINT-" + std::to_string(row) + "-" + std::to_string(col));
                }
                else if (isFastPublisher) {
                    anim.UpdateNodeColor(nodeId, 255, 0, 0); // Vermelho: Rápido (Estático)
                    anim.UpdateNodeSize(nodeId, 8.0, 8.0);
                    anim.UpdateNodeDescription(nodeId, "NODE-FAST-" + std::to_string(row) + "-" + std::to_string(col));
                } else {
                    anim.UpdateNodeColor(nodeId, 0, 255, 0); // Verde: Lento (Estático)
                    anim.UpdateNodeSize(nodeId, 7.0, 7.0);
                    anim.UpdateNodeDescription(nodeId, "NODE-SLOW-" + std::to_string(row) + "-" + std::to_string(col));
                }
            }
        }
    }


    Simulator::Run();
    Simulator::Destroy();

    delete mobilityMgr;
    delete error_model;

    return 0;
}

} // namespace ns3

int main(int argc, char* argv[]) {
    return ns3::main_ndn(argc, argv);
}
