#define voltage_divider 5           // input voltage to the voltage divider #define strain_guage_factor 15 // guage factor
#define initial_resistance 1000000 // source resistance to the voltage divider #define initial_strain_resistance 349000 // ideal condition, strain resistance #define intitialresistance 349 // strain resistance in MΩ


void setup() {
// put your setup code here, to run once:

pinMode(35,INPUT); Serial.begin(9600);

}

void loop()
{
// put your main code here, to run repeatedly Serial.print("Welcome to");
  
delay(100);
Serial.print("Development of self powered wireless strain sensor for Structural Health Monitoring");
// Reading strain sensor values using ping number into the Esp32 int voltage_value = analogRead(35);
float vout= (voltage_value) * (3.3/4095);//conversion of output voltage from the voltage divider
delay(100);
Serial.print("Output voltage from voltage divider circuit in(volts) :");// printing output voltage on the serial monitor
delay(100);
Serial.println( vout);// printing output voltage in the serial monitor delay(100);
Serial.print("volts"); delay(100);

// Obtaining resistance of the Nanocomposite Sensor by simple calculations

float strain_resistance =((vout*initial_resistance)/(voltage_divider - vout)/1000);// strain sensor resistance
Serial.print("resistance in(KΩ) : "); delay(1000);
Serial.println(strain_resistance);// printing Nanocomposite sensor values in MΩ Serial.print("MΩ");

// calculations of ∆R(change in resistance)
 

float changein_resistance = intitialresistance - strain_resistance;

delay(100);
Serial.print("change in resistance in(MΩ): "); Serial.println(changein_resistance);// printing resistance change (∆R) in KΩ delay(100);

// calculations of Strain

float strain1 = (((changein_resistance) /(initial_strain_resistance *strain_gauge_factor ))*100); 
float strain = strain1 * 100;
delay(100); 
Serial.print("strain in(%):");
Serial.println(strain); // printing strain in percentage on the serial monitor delay(100);

if(strain_resistance > (320))
{
delay(100);
Serial.print("Load is applied");
}
else
{
Serial.print("Load is not applied"); delay(1000);
}

}
