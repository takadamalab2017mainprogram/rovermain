#include <algorithm>
#include "debug.h"
#include "task.h"


TaskBase::TaskBase() : mName(""),mPriority(UINT_MAX),mInterval(UINT_MAX),mSlept(0),mIsRunning(false)
{
	TaskManager* pManager = TaskManager::getInstance();
	pManager->add(this);
}
TaskBase::~TaskBase()
{
	TaskManager* pManager = TaskManager::getInstance();
	pManager->del(this);
}


void TaskBase::setRunMode(bool running)
{
	if(running)
	{
		if(mIsRunning == false)init();
		mIsRunning = running;
		mSlept = 0;
	}else
	{
		if(mIsRunning == true)clean();
		mIsRunning = running;
		mSlept = 0;
	}
}
void TaskBase::setName(const char* name)
{
	if(name == NULL)
	{
		Debug::print(LOG_DETAIL ,"Task name is NOT specified!\r\n");
		return;
	}
	Debug::print(LOG_DETAIL ,"Task name is %s!\r\n",name);
	mName = std::string(name);
}
void TaskBase::setPriority(unsigned int pri,unsigned int interval)
{
	mPriority = pri;
	mInterval = interval;
	Debug::print(LOG_DETAIL ,"Task priority is %d %d!\r\n",mPriority,mInterval);
}
bool TaskBase::init()
{
	return true;
}

void TaskBase::clean()
{
}

bool TaskBase::command(const std::vector<std::string> args)
{
	Debug::print(LOG_DETAIL,"Command Not Found!\r\n");
	return true;
}

void TaskBase::update()
{
}

TaskManager::TaskManager()
{
}
TaskManager::~TaskManager()
{
	clean();
}
TaskManager* TaskManager::getInstance()
{
	static TaskManager signleton;
	return &signleton;
}

bool TaskManager::init()
{
	mTasks.clear();
	return true;
}
void TaskManager::clean()
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while(it != mTasks.end())
	{
		if(*it != NULL)(*it)->clean();
		++it;
	}
	mTasks.clear();
}

bool TaskManager::command(const std::vector<std::string> args)
{
	if(args.size() != 0)
	{
		TaskBase* pTask = get(args[0]);
		if(pTask != NULL)
		{
			//コマンド実行対象のタスクが見つかったらコマンドを実行
			if(pTask->mName.compare(args[0]) == 0)
			{
				if(pTask->mIsRunning)
				{
					if(pTask->command(args))return true;
				}else
				{
					//アクティブではないタスクの場合
					Debug::print(LOG_MINIMUM, "This task is not working.");
					return false;
				}
			}
		}
		Debug::print(LOG_MINIMUM, "Command Not Found\r\n");
	}
	return false;
}
void TaskManager::update()
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while(it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if(pTask != NULL)
		{
			//実行するタイミングであれば処理を行う(mIntervalがUINT_MAXならupdate不要なタスク)
			if(pTask->mInterval != UINT_MAX && pTask->mIsRunning && (pTask->mInterval <= pTask->mSlept++))
			{
				pTask->update();
				pTask->mSlept = 0;
			}
		}
		++it;
	}
}

TaskBase* TaskManager::get(const std::string name)
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while(it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if(pTask != NULL)
		{
			if(pTask->mName.compare(name) == 0)
			{
				return pTask;
			}
		}
		++it;
	}
	return NULL;
}

void TaskManager::add(TaskBase* pTask)
{
	if(pTask == NULL)
	{
		Debug::print(LOG_MINIMUM ,"TaskManager(add): No Task Specified!\r\n");
		return;
	}
	if(find(mTasks.begin(),mTasks.end(), pTask) == mTasks.end())
	{
		//すでに追加されていなければタスクを追加する
		mTasks.push_back(pTask);
		sortByPriority();
	}
}
void TaskManager::del(TaskBase* pTask)
{
	if(pTask == NULL)
	{
		Debug::print(LOG_MINIMUM ,"TaskManager(del): No Task Specified!\r\n");
		return;
	}
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while(it != mTasks.end())
	{
		if(pTask == *it)
		{
			*it = NULL;
			Debug::print(LOG_DETAIL ,"TaskManager(del): Succeeded!\r\n");
			return;
		}
		++it;
	}
	Debug::print(LOG_MINIMUM ,"TaskManager(del): Task Not Found!\r\n");
}
void  TaskManager::sortByPriority()
{
	std::sort(mTasks.begin(),mTasks.end(),TaskSoter());
}
