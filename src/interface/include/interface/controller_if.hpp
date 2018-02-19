/* 
 * controller_if.hpp
 * 
 * Created on: Jun 19, 2017
 * Description: generic controller interface  
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef CONTROLLER_IF_HPP
#define CONTROLLER_IF_HPP

namespace librav
{

template<typename CtrlParamType, typename StateType, typename InputType, typename OutputType>
class ControllerInterface
{
public:
	ControllerInterface(const StateType& state):
		state_(state),initialized_(false){};
	virtual ~ControllerInterface() = default;

	typedef CtrlParamType ParamType;

	virtual void InitParams(const CtrlParamType& param) = 0;
	virtual void Update(const InputType& desired, OutputType* cmd) = 0;

protected:
	const StateType& state_;
	CtrlParamType param_;
	bool initialized_;
};

}

#endif /* CONTROLLER_IF_HPP */
