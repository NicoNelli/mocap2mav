#ifndef EXECUTIONER_H
#define EXECUTIONER_H
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include <string>
#include "common/MavState.h"
#include "common/common.h"
#include <deque>
#include "common/MavState.h"



class Executioner{

public:

	Executioner();
	void run();
	bool CheckActions(int a);
	bool readyToPublish();
	void setNextTask(const exec::task task);
	void setLastTask(const exec::task task);
	void clearList();
    void abort();
	void rtl();
    void loadTask();
	void setState(MavState s);
    void init();
	void setList(std::deque<exec::task> list);
	void setStatus(bool armed, bool landed);
	exec::task _actualTask;
	bool _can_run;
	int _actualNode;
	bool _idle;

private:

	MavState _state;
	std::deque<exec::task> _nodeList;
	bool _newTask;
	bool _publish_task;

	bool _armed;
	bool _landed;

};
#endif
		
