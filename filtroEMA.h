//Filtro Media Móvil como Pasa Bajos
//An=a*M+(1-a)*An
//alpha 1: Sin filtro
//alpha 0: Filtrado totalmente
//alpha clásico 0.05


class EMA{
  double alpha,adc_filtrado,adc_raw;
  int n=0;
  public:
    EMA();
    EMA(double);
    double media(double valor);
};
EMA::EMA(){//por defecto
 adc_filtrado = 0;
 adc_raw = 0;
 alpha =0.5;
}
EMA::EMA(double a){//definido por el usuario
 adc_filtrado = 0;
 adc_raw = 0;
 alpha =a;
}
double EMA::media(double valor){
   
   adc_raw = valor;//se lee el valor del sensor
   adc_filtrado = (alpha*adc_raw) + ((1-alpha)*adc_filtrado);//se calcula la media
   if(n>10){
    adc_filtrado=valor;
    n=0;
   }
   n++;
   return adc_filtrado;
}
