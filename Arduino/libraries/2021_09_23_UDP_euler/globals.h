/**************************************************************************/
/*
    Global variables
*/
/**************************************************************************/

#ifndef GLOBALS
#define GLOBALS

#define ID_SERVER 0
#define ID_MOTOR1 1
#define ID_MOTOR2 2

#define N_CARACS 3
#define N_SIZE 10


typedef struct {
  float f_data [N_CARACS];
  float f_data_old [N_CARACS];

  uint16_t l_data;
  uint16_t l_data_old;

  uint16_t l_time;
  uint16_t l_time_old;

} buffer_data;


int Who_Am_I = -1;


/**************************************************************************/
/*
    Utilities
*/
/**************************************************************************/

bool compare(char *val, int len, char *target)
{
  for (int i = 0; i < len; i++)
  {
    if (target[i] != val[i])
    {
      return false;
    }
  }
  return true;
}


bool compare(byte *val, int len, byte *target)
{
  for (int i = 0; i < len; i++)
  {
    if (target[i] != val[i])
    {
      return false;
    }
  }
  return true;
}

void data_to_char_buffer(buffer_data data, int len, char *buff)
{
  String my_string;
  String sum = String("UDP_begin;");

  for (int i = 0 ; i < N_CARACS; i++)
  {
    my_string = String(data.f_data[i] * 1000.0 , 2);
    sum.concat(my_string);
    sum.concat(";");
  }

  my_string = String(data.l_data, DEC);
  sum.concat(my_string);
  sum.concat(";");
  sum.concat(millis());

  sum.concat(";UDP_end");

  sum.toCharArray(buff, len);
}

bool char_buffer_to_data(char *buff, int len, buffer_data &data)
{
  String my_string;
  String sum = String(buff);
  sum.trim();
  int pos = sum.indexOf("UDP_begin;");

  if (pos == -1)
  {
    Serial.println("invalid data");
    return false;
  }
  else
  {
    sum.remove(0, pos + 10);

    for (int i = 0; i < N_CARACS; i++)
    {
      pos = sum.indexOf(";");
      if (pos == -1)
      {
        Serial.println("invalid data");
        return false;
      }
      my_string = sum.substring(0, pos);
      sum.remove(0, pos + 1);
      data.f_data[i] = my_string.toDouble() / 1000.0;
    }
    pos = sum.indexOf(";");
    if (pos == -1)
    {
      Serial.println("invalid data");
      return false;
    }
    my_string = sum.substring(0, pos);
    sum.remove(0, pos + 1);
    data.l_data = my_string.toInt();

    pos = sum.indexOf(";");
    if (pos == -1)
    {
      Serial.println("invalid data");
      return false;
    }
    my_string = sum.substring(0, pos);
    sum.remove(0, pos + 1);
    data.l_time = my_string.toInt();

    pos = sum.indexOf("UDP_end");
    if (pos == -1)
    {
      Serial.println("invalid data");
      return false;
    }

  }
  return true;
}





void overwrite(char *val, int len, char *target)
{
  for (int i = 0; i < len; i++)
  {
    target[i] = val[i];
  }
}

void overwrite(byte *val, int len, byte *target)
{
  for (int i = 0; i < len; i++)
  {
    target[i] = val[i];
  }
}



/*

  void double_to_char_array(double val, char *target)
  {
  char myChar[N_SIZE];
  String my_string = String(val, N_SIZE - 4);
  my_string.toCharArray(myChar , N_SIZE);
  overwrite(myChar, N_SIZE, target);
  }

  double char_array_to_double(char *char_10)
  {
  double rep_val = 0.0;
  rep_val = strtod(char_10, NULL);
  return rep_val;
  }
*/

#endif
