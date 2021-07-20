//
// Created by niew on 2021/02/22.
//
#ifndef FILTER_QUEUE_H
#define FILTER_QUEUE_H

#include <iostream>
#include <queue>
#include <mutex>
#include "filterCommon.h"

namespace filter
{

	class filterQueue
	{
	public:
		filterQueue()
		{
		}

		~filterQueue()
		{
		}

		bool push_back(const observedMessagePtr& pMessage)
		{
			bool bRet = false;
			if (pMessage != NULL)
			{
				m_Mutex.lock();
				mFilterQueue.push(pMessage);
				m_Mutex.unlock();
			}
			return bRet;
		}

		bool pop_up(observedMessagePtr& message)
		{
			bool bRet = false;
			m_Mutex.lock();
			message = mFilterQueue.top();
			mFilterQueue.pop();
			m_Mutex.unlock();
			return bRet;
		}

		int size()
		{
			m_Mutex.lock();
			int size = mFilterQueue.size();
			m_Mutex.unlock();
			return size;
		}

		bool clear()
		{
			bool bRet = true;
			m_Mutex.lock();
			while (mFilterQueue.size())
				mFilterQueue.pop();
			m_Mutex.unlock();
			return bRet;
		}

		bool empty()
		{
			bool bRet = false;
			m_Mutex.lock();
			if (mFilterQueue.empty())
			{
				bRet = true;
			}
			m_Mutex.unlock();
			return bRet;
		}

	private:
		int m_ifilterQueueSize;
		int m_ifilterQueueIndex;

		std::mutex m_Mutex;
		std::priority_queue<observedMessagePtr, std::vector<observedMessagePtr >, observedMessage> mFilterQueue;
	};

}

#endif