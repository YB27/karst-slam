#ifndef GRAPHSLAMENGINEVIEWER_H
#define GRAPHSLAMENGINEVIEWER_H

#include "karst_slam/typedefs.h"
#include "karst_slam/Observable.h"

#include <memory>
#include <vector>
#include <functional>
#include <memory>
#include <experimental/propagate_const>

// Forward declarations
namespace karst_slam
{
    namespace slam{class GraphSlamEngine;}
    namespace gui{class GraphSlamEngineViewer_impl;}
}
namespace mrpt
{
    namespace utils{class CConfigFile;}
    namespace obs
    {
        class CSensoryFramePtr;
        class CObservationPtr;
    }
}

namespace karst_slam{namespace gui{
/** Extends the simulationViewer used to display merged data (display estimated surface and elevation angle distributions)
 *  for all the pair of scans during the construction of the graph slam
 */
class GraphSlamEngineViewer : public karst_slam::Observer
{
public:
    GraphSlamEngineViewer(const mrpt::utils::CConfigFile& cfg,
                          const std::string& name = "GraphSlamEngineViewer");
    GraphSlamEngineViewer(const GraphSlamEngineViewer&) = delete;
    ~GraphSlamEngineViewer();

    GraphSlamEngineViewer& operator=(const GraphSlamEngineViewer&) = delete;

    void queryKeyboardEvents();
    bool isExit()const;

    // Only used in step-by-step replay
    void pause();
    bool isPause()const;

    void dumpKeystrokesToConsole() const;

    void updateFrom(Observable *s) override;

private :
   std::experimental::propagate_const<std::unique_ptr<GraphSlamEngineViewer_impl>> m_pImpl;

};
}} // end namespaces

#endif //GRAPHSLAMENGINEVIEWER_H
