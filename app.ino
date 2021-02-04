#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

//.................variaveis a ser alimentada pelo usuario..............................................

//quantidade de espira programada para rebobinar
unsigned int quant_esp = 40;

//tamanho do carretel a ser rebobinado em mm
float tam_carretel = 20;
//controle se o valor for 1 o valor sera a seccao se for 0 sera o diametro
bool control_s_d = 0;
//medida do fio a ser utilizado diametro ou seccao que sera setado na variavel control_s_d
float fio_sec_diam = 1.25; //= 0.159;
//controle de parada se for 1 modo parar se 0 continua
bool control_parada = 0;
//indica qual o valor em mm que sera avancado em relacao ao sensor de referencia
float avanco = 10.0;
//seta o valor da velocidade de rotacao dos motores
uint8_t velocidade ;

//............................variaveis do sistema.......................................................

//tamanho do passo do fuso do motor 2 em milimetro por volta
const float passo_fuso = 2;
//numero de passo por volta do motor 1
const unsigned int n_passo_volta_m1 = 400;
//numero de passo por volta do motor 2
const unsigned int n_passo_volta_m2 = 200;
//largura de cada passo do motor2 e em milimetro
const float resolucao_passo_m2 =  passo_fuso / n_passo_volta_m2;
//quantidade de passo que o motor 1 precisa dar para cada passo do motor 2
unsigned int quant_passo_m1_p_passo_m2;
//pino para controle do passo motor 1
const uint8_t pin_step_m1 = 5; 
//pino para controle da direcao motor 1
const uint8_t pin_dir_m1 = 6;
//pino para controle do passo motor 2
const uint8_t pin_step_m2 = 11;
//pino para controle da direcao motor 2
const uint8_t pin_dir_m2 = 12;
//pino para leitura do sensor de referencia
const uint8_t sensor_ref = 8;
//pino para interrupcao
const uint8_t interrupt = 2;
//variavel que armazena o valor da velocidade convertida
unsigned int vel_convert;
//quantidade de passo do motor 2 em funcao da largura do carretel
unsigned int quant_passo_m2_p_larg_carretel ;
//quantidade e espira em funcao da largura do carretel
unsigned int quant_espira_p_larg_carretel ;
//vairiavel de controle para iniciar a rebobinagem
bool iniciar_reb;
//sinaliza o andamento da rebobinagem se estiver setado em 1 finalizada se 0  interrompida 
bool reb_finalizada;
//quantidade de passo do motor 2 para cada espira rebobinada
 unsigned int quant_passo_m2_p_esp;
          //variavel contador para controle do passo do motor 1
          unsigned int cont_passo_m1;
//variavel contador para controle do passo do motor 2

unsigned int cont_passo_m2_p_c;
//variavel de controle da direcao do motor 2
bool direcao_m2_atual;
unsigned int esp_atual;
unsigned int pass_m1_atual;
unsigned pass_m2_atual;

const int numCoils = 10;
const int numDiscreteInputs = 10;
const int numHoldingRegisters = 10;
const int numInputRegisters = 10;

#define vel_ref 15
#define avancar  true
#define voltar  false
//prototipo de funcoes...............
void rebobinar();
void iniciar();
void parar();
void interrupt_parar();
void passo_m1();
void passo_m2(bool dir);
void  referenciar();
void set_velocidade(uint8_t veloc);
void atualizar();
void  atualizar_dados_rec_usuario();

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_dir_m1,OUTPUT);
  pinMode(pin_dir_m2,OUTPUT);
  pinMode(pin_step_m1,OUTPUT);
  pinMode(pin_step_m2,OUTPUT);
  pinMode(interrupt,INPUT_PULLUP);
  pinMode(sensor_ref,INPUT_PULLUP);
  
  //variavel inicializada para teste
  iniciar_reb = true;
  cont_passo_m1 = 0;
  //quant_esp = 10;
  //atualizando valores
  control_parada = 0;
  cont_passo_m2_p_c = 0;
  direcao_m2_atual = true;
  esp_atual = 0;
  pass_m1_atual =0;
  pass_m2_atual =0;
  reb_finalizada = EEPROM.read(20);
  digitalWrite(pin_step_m1,LOW);
  digitalWrite(pin_step_m2,LOW);
 // attachInterrupt(digitalPinToInterrupt(interrupt), interrupt_parar, RISING);
  
     Serial.begin(9600);
  while (!Serial);

 // Serial.println("Modbus RTU Server Kitchen Sink");
    if (!ModbusRTUServer.begin(1, 9600)) {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // configure coils at address 0x00
  ModbusRTUServer.configureCoils(0x00, numCoils);
  // configure discrete inputs at address 0x00
  ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);
  // configure holding registers at address 0x00
  ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);
  // configure input registers at address 0x00
  ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);
   
   //setando bit de controle de andamento da rebobinagem em modo parado
   ModbusRTUServer.coilWrite(0,true);
   //inicializando registro com o valor da seccao do fio
  // ModbusRTUServer.holdingRegisterWrite(0,53477);
  // ModbusRTUServer.holdingRegisterWrite(1,15906);
  
   //setando quantidade de espira
  // ModbusRTUServer.holdingRegisterWrite(6,40);
   //setando velocidade
   //ModbusRTUServer.holdingRegisterWrite(7,20);
   //setando bit de controle da seccao ou diametro 
    //ModbusRTUServer.coilWrite(2,false);

     //atualizar_dados_rec_usuario();
     // referenciar();

} 
    
   void referenciar(){
     unsigned int temp = 0;
     while(!digitalRead(sensor_ref)){
       set_velocidade(vel_ref);
       passo_m2(false);
        
     }
     if(reb_finalizada){
       temp = (avanco/resolucao_passo_m2);
      }else{
        atualizar();
        temp = (unsigned int)(avanco/resolucao_passo_m2) + cont_passo_m2_p_c ;
      }
     for(unsigned int i = 0; i < temp ;i++){
           set_velocidade(vel_ref);
           passo_m2(true);
     }
     
   }
   
 void salvar(){
      uint8_t temp[4] ;
      EEPROM.write(0,direcao_m2_atual);

      EEPROM.write(1,pass_m1_atual);
      EEPROM.write(2,(pass_m1_atual>>8));

      EEPROM.write(3,pass_m2_atual);
      EEPROM.write(4,(pass_m2_atual>>8));

      EEPROM.write(5,esp_atual);
      EEPROM.write(6,(esp_atual>>8));

      EEPROM.write(7,cont_passo_m2_p_c);
      EEPROM.write(8,(cont_passo_m2_p_c>>8));

      EEPROM.write(9,quant_esp);
      EEPROM.write(10,(quant_esp>>8));
       
      memcpy(&temp,&fio_sec_diam,4);        
      EEPROM.write(11,temp[0]);
      EEPROM.write(12,temp[1]);
      EEPROM.write(13,temp[2]);
      EEPROM.write(14,temp[3]);
      
      int teste1,teste2;
        memcpy(&teste1,&temp[0],2); 
        memcpy(&teste2,&temp[2],2);
        ModbusRTUServer.holdingRegisterWrite(0,teste1);
        ModbusRTUServer.holdingRegisterWrite(1,teste2);

      memcpy(&temp,&tam_carretel,4);
      EEPROM.write(15,temp[0]);
      EEPROM.write(16,temp[1]);
      EEPROM.write(17,temp[2]);
      EEPROM.write(18,temp[3]);
      
      EEPROM.write(19,control_s_d);

      EEPROM.write(20,reb_finalizada);

      memcpy(&temp,&avanco,4);        
      EEPROM.write(21,temp[0]);
      EEPROM.write(22,temp[1]);
      EEPROM.write(23,temp[2]);
      EEPROM.write(24,temp[3]);

      EEPROM.write(25,velocidade);
      EEPROM.write(26,(velocidade>>8));
      
      }
         
   void atualizar(){
      uint8_t temp[4] ;
     // float temp2;
      control_s_d = EEPROM.read(19);
      direcao_m2_atual = EEPROM.read(0);
      pass_m1_atual = (EEPROM.read(1) | (EEPROM.read(2)<<8));
      pass_m2_atual = (EEPROM.read(3) | (EEPROM.read(4)<<8));
      esp_atual =  (EEPROM.read(5) | (EEPROM.read(6)<<8));
      cont_passo_m2_p_c =  (EEPROM.read(7) | (EEPROM.read(8)<<8));
      quant_esp =  (EEPROM.read(9) | (EEPROM.read(10)<<8));
      velocidade =  (EEPROM.read(25) | (EEPROM.read(26)<<8));

      temp[0] = EEPROM.read(11);
      temp[1] = EEPROM.read(12);
      temp[2] = EEPROM.read(13);
      temp[3] = EEPROM.read(14);
      memcpy(&fio_sec_diam,&temp, 4);
      //fio_sec_diam = temp2;

      temp[0] = EEPROM.read(15);
      temp[1] = EEPROM.read(16);
      temp[2] = EEPROM.read(17);
      temp[3] = EEPROM.read(18);
      memcpy(&tam_carretel,&temp, 4);
      //tam_carretel = temp2;

      temp[0] = EEPROM.read(21);
      temp[1] = EEPROM.read(22);
      temp[2] = EEPROM.read(23);
      temp[3] = EEPROM.read(24);
      memcpy(&avanco,&temp, 4);
      //avanco = temp2;
     }

      void atualizar_dados_rec_usuario(){
       uint8_t temp[4] ;
       long reg1,reg2;
       
       reg1 = ModbusRTUServer.holdingRegisterRead(0);
       reg2 = ModbusRTUServer.holdingRegisterRead(1);
       memcpy(&temp[0],&reg1,2);
       memcpy(&temp[2],&reg2,2);
       memcpy(&fio_sec_diam,&temp,4);
       /*
       reg1 = ModbusRTUServer.holdingRegisterRead(2);
       reg2 = ModbusRTUServer.holdingRegisterRead(3);
       memcpy(&temp[0],&reg1,2);
       memcpy(&temp[2],&reg2,2);
       memcpy(&avanco,&temp,4);

       reg1 = ModbusRTUServer.holdingRegisterRead(4);
       reg2 = ModbusRTUServer.holdingRegisterRead(5);
       memcpy(&temp[0],&reg1,2);
       memcpy(&temp[2],&reg2,2);
       memcpy(&tam_carretel,&temp,4);
       */
      reg1 = ModbusRTUServer.holdingRegisterRead(6);
      memcpy(&quant_esp,&reg1,2);
      
      reg1 = ModbusRTUServer.holdingRegisterRead(7);
      memcpy(&velocidade,&reg1,1);

      control_s_d = ModbusRTUServer.coilRead(2);

     }
       void atual_dados_online(){
       long reg1;
       reg1 = ModbusRTUServer.holdingRegisterRead(7);
       memcpy(&velocidade,&reg1,1);
       }

      void set_velocidade(uint8_t veloc){
          if((veloc > 0) && (veloc <21)){
               vel_convert = (21 - veloc);
          }else{
               vel_convert =  5;
          }
             
      }

  void configuracao(){
     float diametro_fio;
   //diametro do fio usado para rebobinar
  
  if(control_s_d){
    diametro_fio = 2 * sqrt(fio_sec_diam/PI);
  }else{
    diametro_fio = fio_sec_diam;
  }
  
  quant_espira_p_larg_carretel = (tam_carretel / diametro_fio);
  quant_passo_m2_p_esp = (diametro_fio / resolucao_passo_m2); //50
  quant_passo_m2_p_larg_carretel = quant_passo_m2_p_esp * quant_espira_p_larg_carretel; //100
  quant_passo_m1_p_passo_m2 = n_passo_volta_m1/quant_passo_m2_p_esp;
  
   //seleciona a direcao
    digitalWrite(pin_dir_m1,HIGH);
    set_velocidade(velocidade);
      //referenciar();

 }
 void passo_m1(){
       digitalWrite(pin_dir_m1,0);
      //avanca um passo no motor 1
      digitalWrite(pin_step_m1,HIGH);
      delayMicroseconds(500);
      digitalWrite(pin_step_m1,LOW);
      delay(vel_convert);
      //digitalWrite(pin_step_m1,!digitalRead(pin_step_m1));
      //cont_passo_m1++;
 }

 void passo_m2(bool dir){
      //seleciona a direcao
      digitalWrite(pin_dir_m2,dir);
      //avanca um passo no motor 2
      digitalWrite(pin_step_m2,HIGH);
      delayMicroseconds(500);
      digitalWrite(pin_step_m2,LOW);
      delay(vel_convert);
     // cont_passo_m2_p_c++;
     // Serial.print( cont_passo_m2_p_c);
      
 }
    void iniciar(){
    
     //verificar se a rebobinagem esta parada 
     if(!ModbusRTUServer.coilRead(0)){
       //verifica se tem rebobinagem nao concluida 
       if(EEPROM.read(20)){
         referenciar();
        // atualizar_dados_rec_usuario();
         salvar();
         configuracao();
         
       }else{
        atualizar();
        configuracao();
       }
       rebobinar();
        //Serial.print('1');
    }
    }
   
     void finalizar(){
       //Serial.print("  finalizada");
      reb_finalizada = true;
      control_parada = true;
      ModbusRTUServer.coilWrite(0,true);
      cont_passo_m2_p_c = 0;
      direcao_m2_atual = true;
      esp_atual = 0;
      pass_m1_atual =0;
      pass_m2_atual =0;
       salvar();
      
     }


    
     
    void rebobinar(){
           // unsigned int pass_interval = 0 ;
            
           //for( ; esp_atual < quant_esp ; esp_atual++){
              while((esp_atual < quant_esp) && (ModbusRTUServer.coilRead(0) == false)){
                                              
               //Serial.print(((quant_esp * n_passo_volta_m1)/quant_passo_m1_p_passo_m2));
               while((pass_m1_atual < n_passo_volta_m1) && (ModbusRTUServer.coilRead(0) == false)){
               
               while((pass_m2_atual < quant_passo_m1_p_passo_m2) && (pass_m1_atual < n_passo_volta_m1) && (ModbusRTUServer.coilRead(0) == false)){
               pass_m2_atual ++;  
               pass_m1_atual++; 
              //for( ; pass_m1_atual < n_passo_volta_m1; pass_m1_atual++){
                
                ModbusRTUServer.poll();
                atual_dados_online();
                set_velocidade(velocidade);
                passo_m1();
                
                           
              }
               if(pass_m2_atual  >= quant_passo_m1_p_passo_m2){
                    pass_m2_atual  = 0;
                 } 
               
            // if(ModbusRTUServer.coilRead(0) == true){
              if(direcao_m2_atual){
                // while((pass_m2_atual < quant_passo_m2_p_esp) && (pass_m1_atual >= n_passo_volta_m1) && (ModbusRTUServer.coilRead(0) == false)){
                 //pass_m2_atual++;
            
                   ModbusRTUServer.poll();
                   atual_dados_online();
                   set_velocidade(velocidade);
                   passo_m2( direcao_m2_atual);
                   cont_passo_m2_p_c++;
                 
                 // }
                   if((cont_passo_m2_p_c >=  quant_passo_m2_p_larg_carretel)){
                     direcao_m2_atual = false;
                   }
                   
              }else{
                 //while((pass_m2_atual < quant_passo_m2_p_esp) && (pass_m1_atual >= n_passo_volta_m1) && (ModbusRTUServer.coilRead(0) == false)){
                 //pass_m2_atual++;
                   
                  ModbusRTUServer.poll();
                  atual_dados_online();
                  set_velocidade(velocidade);
                  passo_m2( direcao_m2_atual);

                   if(cont_passo_m2_p_c > 0){
                    cont_passo_m2_p_c--;
                                 
                  }else{
                     direcao_m2_atual = true;
                  }

              
              }
             //}

          }
               if(pass_m1_atual >= n_passo_volta_m1){
                    pass_m1_atual = 0;
                    pass_m2_atual  = 0;
                    esp_atual++;
                 }
              
              //*/
             
              }
          if((ModbusRTUServer.coilRead(0) == false)){
          finalizar();
          }else{
          //control_parada = 1;
           reb_finalizada = false;
           salvar();
         
          }
    }
       
// unsigned int teste  = 10;
void loop() {
  // salvar();
   //atualizar();
 
  
   ModbusRTUServer.poll();
  
  // map the coil values to the discrete input values
     //ModbusRTUServer.holdingRegisterWrite(7,20);
    // ModbusRTUServer.coilWrite(0,0);
   //control_parada = 1;
   //salvar();
   //configuracao();
   /*
     Serial.print(quant_passo_m1_p_passo_m2);
     Serial.print("   ");
     Serial.print(fio_sec_diam);
      Serial.print("   ");
     Serial.print(diametro_fio);
      */
    iniciar();
   //rebobinar();
     // Serial.print(quant_passo_m1_p_passo_m2);
   //rebobinar();
   //delay(5000);
   //passo_m2(0);
    
}