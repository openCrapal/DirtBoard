/*
  reg.h - Library for controlling physical quantitys using PID algo and arduino
  Created by Ulysse R. Delplanque, Mars 26, 2017.
  Released into the public domain.
*/


#ifndef reg_h
#define reg_h
//#include "Arduino.h"


class Block
{
  public:
	Block(float val);
	Block();
    float getVal();
    void setVal(float val);
	static void newCycle();
	static long _current_index;
	long _index;
  protected:
    float _val;
    virtual void _update();
    long _t_old;
};

class I_Block : public Block
{
  public:
    I_Block(Block *inputBlock);
  protected:
    void _update();
	Block *_inputBlock;
};

class D_Block : public Block
{
  public:
    D_Block(Block *inputBlock);
  protected:
    void _update();
	float _oldVal;
	Block *_inputBlock;
};

class S_Block : public Block
{
  public:
    S_Block(Block *inputBlock0, Block *inputBlock1);
	S_Block(Block *inputBlock0, Block *inputBlock1, float g0, float g1);
  protected:
    void _update();
	Block *_inputBlock0;
	Block *_inputBlock1;
	float _g0;
	float _g1;
};

class PID_Block : public Block
{
  public:
    PID_Block(Block *feedBack, float kp, float ki, float kd, float sat);
	PID_Block(Block *feedBack, Block *valRef, float kp, float ki, float kd, float sat);
  protected:
    void _update();
	float _old_I;
	float _sat;
	float _kp;
	S_Block *_delta;
	I_Block *_I;
	S_Block *_PID;

};

#endif
