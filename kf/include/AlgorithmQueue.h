#ifndef ALGORITHM_QUEUE_H
#define ALGORITHM_QUEUE_H

#include "filterCommon.h"

#if BUILD_MPU
#include "KtMutex.h"
#include "ModuleBase.h"
#include "AnalyticalCommon.h"
#include "LaneLocationCommon.h"

#else
#include <mutex>          // std::mutex
#endif


namespace filter
{

#define MAX_NUM  50

class AlgorithmQueue
{
    public: 
    
    AlgorithmQueue();
    ~AlgorithmQueue();

    bool push_back(MeasurementPtr& pMessage);
    bool pop_up(MeasurementPtr& message);
    int  size();
    bool clear();
    bool empty();

    private:

#if BUILD_MPU
        tool_t::KTMutex_t m_Mutex;
#else
        std::mutex m_Mutex;
#endif
    int                         m_iAlgorithmQueueSize;
    int                         m_iAlgorithmQueueIndex;
    MeasurementPtr m_AlgorithmQueue[MAX_NUM];
};

}

#endif