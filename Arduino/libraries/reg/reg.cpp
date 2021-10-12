/*
  reg.cpp - Library for controlling physical quantities using PID algorithm and Arduino
  Created by Ulysse R. Delplanque, Mars 26, 2017.
  Released into the public domain.
*/
#include "reg.h"
#include "Arduino.h"


// Static variables
long Block::_current_index = 0;


// The original, the mum class, the Block
Block::Block()
{
	_val = 0.0;
	_index = 0;
    _t_old = millis();
}

Block::Block(float val) : Block()
{
	_val = val;
}

void Block::newCycle()
{
	_current_index ++;
}

void Block::setVal(float val)
{
	_val = val;
}

void Block::_update()
{
	//Nothing
}

float Block::getVal()
{
	if (_index < _current_index)
	{
		//this->_update();
		_update();
		_index = _current_index;
		_t_old = millis();
	}
	return _val;
}

// Integrator, I_Block
I_Block::I_Block(Block *inputBlock) : Block(), _inputBlock(inputBlock)
{
}

void I_Block::_update()
{
	float dt = (millis() - _t_old) * 0.001;
	_val += _inputBlock->getVal() * dt;
}

// Differential, D_Block
D_Block::D_Block(Block *inputBlock) : Block(), _inputBlock(inputBlock), _oldVal(0.0)
{
}

void D_Block::_update()
{
	long t = millis();
	if(t == _t_old)
	{
		t++;
	}
	float dt = (t - _t_old) * 0.001;
	_val = (_inputBlock->getVal() -_oldVal) / dt;
	_oldVal = _inputBlock->getVal();
}

// Sum, S_Block
S_Block::S_Block(Block *inputBlock0, Block *inputBlock1) : Block(),
		_inputBlock0(inputBlock0), _inputBlock1(inputBlock1), _g0(1.),  _g1(1.)
{
}
S_Block::S_Block(Block *inputBlock0, Block *inputBlock1, float g0, float g1) : S_Block(inputBlock0, inputBlock1)
{
	_g0 = g0;
	_g1 = g1;
}

void S_Block::_update()
{
	_val = _g0*_inputBlock0->getVal() + _g1*_inputBlock1->getVal();
}


/* ============================== PID 1DoF ============================ */
PID_Block::PID_Block(Block *feedBack, Block *valRef, float kp, float ki, float kd, float sat): Block()
{
	_kp = kp;
	_sat = sat;
	_old_I = 0.;
	_delta = new S_Block(valRef, feedBack, 1., -1.);
	_I = new I_Block(_delta);
	_PID = new S_Block(_delta, new S_Block(_I, new D_Block(_delta), ki, kd), kp, 1.);
}

PID_Block::PID_Block(Block *feedBack, float kp, float ki, float kd, float sat): PID_Block(feedBack, new Block(0), kp, ki, kd, sat)
{
}


void PID_Block::_update()
{
	_val = _PID->getVal();
	if (_val>_sat)
	{
		_val = _sat;
		_I->setVal(_sat - _kp * _delta->getVal()); // Anti-windup
	}
	else if (_val<-_sat)
	{
		_val = -_sat;
		_I->setVal(-_sat - _kp * _delta->getVal()); // Anti-windup
	}
}

