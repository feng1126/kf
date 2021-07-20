#include "AlgorithmQueue.h"

namespace filter
{

    AlgorithmQueue::AlgorithmQueue()
    {
        m_iAlgorithmQueueSize  = 0;
        m_iAlgorithmQueueIndex = 0;
    }

    AlgorithmQueue::~AlgorithmQueue()
    {
        
    }

    bool AlgorithmQueue::push_back(MeasurementPtr &pMessage)
    {
        bool bRet = false;
        if( pMessage != NULL )
        {
            m_Mutex.lock();
            if( m_iAlgorithmQueueSize < MAX_NUM )
            {
                int iPushIndex = (m_iAlgorithmQueueSize + m_iAlgorithmQueueIndex) % MAX_NUM;
                m_AlgorithmQueue[iPushIndex] = pMessage;
                m_iAlgorithmQueueSize++;
            }
            else
            {
                int iPushIndex = (m_iAlgorithmQueueSize + m_iAlgorithmQueueIndex) % MAX_NUM;
                m_AlgorithmQueue[iPushIndex] = pMessage;
                m_iAlgorithmQueueIndex++;
                m_iAlgorithmQueueIndex = m_iAlgorithmQueueIndex % MAX_NUM;
                bRet = true;
            }
            // std::cout << " m_iAlgorithmQueueSize : " << m_iAlgorithmQueueSize << std::endl;
            m_Mutex.unlock();
        }        
        return bRet;
    }

    bool AlgorithmQueue::pop_up(MeasurementPtr& message)
    {
        bool bRet = false;
        if( m_iAlgorithmQueueSize > 0 )
        {
            m_Mutex.lock();
            int iPushIndex = (m_iAlgorithmQueueIndex) % MAX_NUM;
            message = m_AlgorithmQueue[iPushIndex];
            m_AlgorithmQueue[iPushIndex].reset();
            m_iAlgorithmQueueIndex++;
            m_iAlgorithmQueueIndex = m_iAlgorithmQueueIndex % MAX_NUM;
            m_iAlgorithmQueueSize--;
            // std::cout << " m_iAlgorithmQueueSize : " << m_iAlgorithmQueueSize << std::endl;
            bRet = true;
            m_Mutex.unlock();
        }
        return bRet;
    }

    int  AlgorithmQueue::size()
    {
        return m_iAlgorithmQueueSize;
    }

    bool AlgorithmQueue::clear()
    {
        bool bRet = true;
        m_Mutex.lock();
        if( m_iAlgorithmQueueSize > 0 )
        {
            m_iAlgorithmQueueSize = 0;
            m_iAlgorithmQueueIndex= 0;
        }
        m_Mutex.unlock();
        return bRet;
    }

    bool AlgorithmQueue::empty()
    {
        bool bRet = false;
        m_Mutex.lock();
        if( m_iAlgorithmQueueSize == 0 )
        {
            bRet = true;
        }
        m_Mutex.unlock();
        return bRet;
    }

}