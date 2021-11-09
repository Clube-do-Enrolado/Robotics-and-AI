/*
 * File:          ARRV9939.c
 * Date:          31/10/2021
 * Description:   Algoritmo para que o robô siga a linha de um percurso.
 * Author:        Andy Barbosa
 *                Rafael Palierini
 *                Rubens Mendes
 *                Vitor Acosta
 *  
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>


// Velocidade máxima, em rad/s, que os
// motores do E-Puck operam.
#define MAX_SPEED 6.28

// Percentual de velocidade que será
// aplicado aos motores
#define MOTOR_INTENSITY 0.9

#define FORWARD_INTENSITY 0.7

#define IR_THRESHOLD 0.1

#define IR_TRACK_VALUE 5

// Motores do E-Puck
static WbDeviceTag left_motor, right_motor;

// Sensores de proximidade
WbDeviceTag ps[8];
// Leitura da saída dos sensores
double ps_values[8];

// Sensores IR (infra-vermelho) para detectar
// a linha do percurso.
static WbDeviceTag left_ground, right_ground, rg_back, lg_back;
// Leitura da saída dos sensores
static double left_ground_value, right_ground_value, rg_back_value, lg_back_value;

// Tempo da simulação
int time_step;


/*
 * Adquire o tempo da simulação.
 */
int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

void step()
{
  if (wb_robot_step(time_step) == -1){
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

//==========================//
//          MOTORES         //
//==========================//

/*
 * Função que define velocidade 0 à ambos motores.
 * Fazendo o robô parar.
 */
void motor_stop(){
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/*
 * Função que define velocidade máxima à ambos motores.
 * Fazendo que o robô vá para frente.
 */
void motor_move_forward(){
  wb_motor_set_velocity(left_motor, FORWARD_INTENSITY*MAX_SPEED);
  wb_motor_set_velocity(right_motor, FORWARD_INTENSITY*MAX_SPEED);
}


/*
 * Função que define velocidade negativa máxima ao motor esquerdo
 * e vlocidade positiva máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a esqueda.
 */
void motor_rotate_left(){
  wb_motor_set_velocity(left_motor, -MOTOR_INTENSITY*MAX_SPEED);
  wb_motor_set_velocity(right_motor, MOTOR_INTENSITY*MAX_SPEED);
}

/*
 * Função que define velocidade positiva máxima ao motor esquerdo
 * e vlocidade negativa máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a direita.
 */
void motor_rotate_right(){
  wb_motor_set_velocity(left_motor, MOTOR_INTENSITY*MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MOTOR_INTENSITY*MAX_SPEED);
}


/////////////////////
//       ROBÔ      //
/////////////////////

/*
 * Método de inicialização do robô.
 * Define os valores de inicialização dos sensores,
 * motores e tempo de simulação.
 */
void init(){

  time_step = get_time_step();
  
  // Nome dos sensores de proximidade
  char ps_names[8][4] ={
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };  

  // Inicialização dos sensores.
  for (int i = 0; i < 8 ; i++) {
    // Liga sensores de proximidade
    ps[i] = wb_robot_get_device(ps_names[i]);
            
    // Ativa todos os sensores, os resultados serão coletados
    // periódicamente em milissegundos TIME_STEP. 
    wb_distance_sensor_enable(ps[i], time_step);
  }
  
  // Inicialização dos motores das rodas esquerda e direita, respectivamente.
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // Inicializa os sensores de IR do solo.
  left_ground = wb_robot_get_device("IR0");
  wb_distance_sensor_enable(left_ground, time_step);
  right_ground = wb_robot_get_device("IR1");
  wb_distance_sensor_enable(right_ground,time_step);
   // Inicializa os sensores de IR do solo.
   
  lg_back = wb_robot_get_device("IR00");
  wb_distance_sensor_enable(lg_back, time_step);
  rg_back = wb_robot_get_device("IR11");
  wb_distance_sensor_enable(rg_back,time_step);
  
}


int main(int argc, char **argv) {
    
  wb_robot_init();
  
  init();
  
  while (true) {
        
    left_ground_value = wb_distance_sensor_get_value(left_ground);
    right_ground_value = wb_distance_sensor_get_value(right_ground);
    
    lg_back_value = wb_distance_sensor_get_value(lg_back);
    rg_back_value = wb_distance_sensor_get_value(rg_back);
    
    printf("\nGround Sensors Values: \n(Left)%4.4f\n(Right)%4.4f\n",
    left_ground_value,
    right_ground_value);
    
    printf("\nBack Ground Sensors Values: \n(Left)%4.4f\n(Right)%4.4f\n",
    lg_back_value,
    rg_back_value);
    
    /*if ( 
     (left_ground_value >= 3 && left_ground_value <= 5) &&
     (right_ground_value >= 3 && right_ground_value <= 5)
     )
      motor_rotate_left();
    else{  */
      motor_move_forward();    
    
      if( (left_ground_value > right_ground_value + 0.5) &&
          (IR_TRACK_VALUE - IR_THRESHOLD < left_ground_value && left_ground_value < 15)){
        printf("\nVirando Esquerda\n");
        motor_rotate_left();
       }
       else if( (right_ground_value > left_ground_value + 0.5) &&
          (IR_TRACK_VALUE - IR_THRESHOLD < right_ground_value && left_ground_value < 15)){
        printf("\nVirando Direita\n");
        motor_rotate_right();
       }
       //}

    step();
  }
}