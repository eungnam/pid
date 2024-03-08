#include <iostream>
#include <bitset>
using namespace std;

#include <stdio.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <limits>

typedef std::numeric_limits< double > dou;
typedef std::numeric_limits< float >  flo;


//_____________________________________________________________________________________________________________________
  double fcn_plant( double m_t)
 {

        double plant_result = 0;
        plant_result = M_PI*sin(2*M_PI*m_t);
        return plant_result;
 }

 double fcn_sensor( double m_yp)
 {
        return m_yp;
 }

int fcn_adc( double m_Plant_InoutRange, int m_AdcR, double m_Vin)
{
        int adc_result = 0;
        adc_result = (int)((m_Vin / (m_Plant_InoutRange)) * ((double)m_AdcR)  );
        return adc_result;
}
//_____________________________________________________________________________________________________________________



//_____________________________________________________________________________________________________________________
double _64_s_en1,_64_s_I_error;
double fcn_s_64( double m_Plant_InoutRange, int m_AdcR, double m_SamT, int m_AdcOut, int i_D_ref, double m_kp, double m_ki, double m_kd, double m_PWM_DuraionT, double m_PWM_IOclkT)
{
     
        double _64en0 = 0;        
        double _64r = 0; 
        double _64yrn = 0;      

        double _64pError = 0;
        double _64dError = 0;    

        double _64_s_Mv = 0;
        //unsigned int long _64Duty;


        //decoder
        _64r       = (m_Plant_InoutRange * i_D_ref) / ((double)m_AdcR);
        _64yrn     = (m_Plant_InoutRange) * ( m_AdcOut / ((double)m_AdcR));
        _64en0     = _64r  - _64yrn;

        //pp
        _64pError       = _64en0;
        _64_s_I_error   = _64_s_I_error + _64en0;        
        _64dError       = _64en0 - _64_s_en1 ;

        _64_s_Mv  = (m_kp*_64pError) + (m_ki*m_SamT*_64_s_I_error) + (m_kd*_64dError/m_SamT);
        //PWM============================================================================================
        //_64Duty = (m_PWM_DuraionT * _64_s_Mv) / m_PWM_IOclkT;
        //================================================================================================

        //end
        _64_s_en1 = _64en0; 
        return _64_s_Mv;   
}



float _32_s_en1,_32_s_I_error;
float fcn_s_32( float m_Plant_InoutRange, int m_AdcR, float m_SamT, int m_AdcOut, int i_D_ref, float m_kp, float m_ki, float m_kd, float m_PWM_DuraionT, float m_PWM_IOclkT)
{
        float _32en0 = 0;  
        float _32r = 0;               
        float _32yrn = 0;    

        float _32Perror = 0;
        float _32Derror = 0;    

        float _32Mv = 0;
        unsigned int _32Duty;

        //decoder
        _32r       = (m_Plant_InoutRange * i_D_ref) / ((float)m_AdcR);
        _32yrn     = (m_Plant_InoutRange) * ( m_AdcOut / ((double)m_AdcR));
        _32en0     = _32r - _32yrn;

        //pp
        _32Perror   = _32en0;
        _32_s_I_error   = _32_s_I_error + _32en0;        
        _32Derror   = _32en0 - _32_s_en1;

        _32Mv  = (m_kp*_32Perror) + (m_ki*m_SamT*_32_s_I_error) + (m_kd*_32Derror/m_SamT);
        //==================================================================================
        //_32Duty = (m_PWM_DuraionT * _32Mv) / m_PWM_IOclkT;
        //=============================================================

        //backup
        _32_s_en1 = _32en0; 
        return _32Mv;   
}

int _ou_s_en1, _ou_s_i_error;
float fcn_s_ou( float m_Plant_InoutRange, int m_AdcR, float m_SamT, int m_AdcOut, int i_D_ref, int m_kp, int m_ki, int m_kd, float m_PWM_DuraionT, float m_PWM_IOclkT)
{
  
        int _OuPerror = 0;
        int _OuDerror = 0;    
        int _ou_s_en0  = 0;

        int _Uint = 0; 

        float _C0 = 0;
        float _C1 = 0;   
        float _OuMv = 0;

        int m_SamF = 0;

        m_SamF = (int)(1/m_SamT);

        _ou_s_en0 = i_D_ref - m_AdcOut;

        _OuPerror       = _ou_s_en0;
        _ou_s_i_error   = _ou_s_i_error + _ou_s_en0;
        _OuDerror       = _ou_s_en0 - _ou_s_en1; 

        _Uint = (m_kp * _OuPerror * m_SamF) + (m_ki * _ou_s_i_error) + (m_kd * _OuDerror * m_SamF * m_SamF); 

        _C0   = 1/((float)m_AdcR*m_SamF);
        _C1   = 2 * 0.031415926535897932384626433832795028841971;

        //=================================================================================
        _OuMv  = (float)_Uint * _C0 * _C1;
        //_OuDuty = (m_PWM_DuraionT * _OuMv) / m_PWM_IOclkT;
        //==================================================================================

        _ou_s_en1  = _ou_s_en0;
        return _OuMv;
}
//_____________________________________________________________________________________________________________________





double _64_d_en2, _64_d_en1, _64_d_Mv1;
double fcn_d_64( double m_Plant_InoutRange, int m_AdcR, double m_SamT, int m_AdcOut, int i_D_ref, double m_kp, double m_ki, double m_kd, double m_PWM_DuraionT, double m_PWM_IOclkT)
{
     
        double _64r = 0; 
        double _64yrn = 0;      
        double _64_d_en0 = 0;

        double _64_d_Mv0 = 0;
        double _64b0=0, _64b1=0, _64b2=0;

        unsigned int long _64Duty;
        //decoder
        _64r       = (m_Plant_InoutRange * i_D_ref) / ((double)m_AdcR);
        _64yrn     = (m_Plant_InoutRange) * ( m_AdcOut / ((double)m_AdcR));
        _64_d_en0     = _64r  - _64yrn;

        //bo,b1,b2
        _64b0    = m_kp + ((m_SamT*m_ki)/2) + (m_kd /m_SamT);
        _64b1    = ((m_ki * m_SamT)/2) - m_kp - ((2*m_kd)/m_SamT);
        _64b2    = m_kd / m_SamT;

        _64_d_Mv0 = _64_d_Mv1 + ( _64b0 * _64_d_en0 ) +  ( _64b1 * _64_d_en1 ) + ( _64b2 * _64_d_en2 );
 
        //PWM============================================================================================
        //_64Duty = (m_PWM_DuraionT * _64_d_Mv0) / m_PWM_IOclkT;
        //================================================================================================

        //end
        _64_d_en2 = _64_d_en1; 
        _64_d_en1 = _64_d_en0; 
        _64_d_Mv1 = _64_d_Mv0;

        return _64_d_Mv0;   
}


float _32_d_en2, _32_d_en1, _32_d_Mv1;
float fcn_d_32( float m_Plant_InoutRange, int m_AdcR, float m_SamT, int m_AdcOut, int i_D_ref, float m_kp, float m_ki, float m_kd, float m_PWM_DuraionT, float m_PWM_IOclkT)
{
     
        float _32r = 0; 
        float _32yrn = 0;      
        float _32_d_en0 = 0;

        float _32_d_Mv0 = 0;
        float _32b0=0, _32b1=0, _32b2=0;

        unsigned int long _32Duty;
        //decoder
        _32r       = (m_Plant_InoutRange * i_D_ref) / ((float)m_AdcR);
        _32yrn     = (m_Plant_InoutRange) * ( m_AdcOut / ((double)m_AdcR));
        _32_d_en0     = _32r  - _32yrn;

        //bo,b1,b2
        _32b0    = m_kp + ((m_SamT*m_ki)/2) + (m_kd /m_SamT);
        _32b1    = ((m_ki * m_SamT)/2) - m_kp - ((2*m_kd)/m_SamT);
        _32b2    = m_kd / m_SamT;

        _32_d_Mv0 = _32_d_Mv1 + ( _32b0 * _32_d_en0 ) +  ( _32b1 * _32_d_en1 ) + ( _32b2 * _32_d_en2 );
 
        //PWM============================================================================================
        //_64Duty = (m_PWM_DuraionT * _64_d_Mv0) / m_PWM_IOclkT;
        //================================================================================================

        //end
        _32_d_en2 = _32_d_en1; 
        _32_d_en1 = _32_d_en0; 
        _32_d_Mv1 = _32_d_Mv0;

        return _32_d_Mv0;  
}




int _ou_d_error1, _ou_d_error2;
float _ou_d_Mv1;
float fcn_d_ou( float m_Plant_InoutRange, int m_AdcR, float m_SamT, int m_AdcOut, int i_D_ref, float m_kp, float m_ki, float m_kd, float m_PWM_DuraionT, float m_PWM_IOclkT)
{
     
        int i_D_error0 = 0;
        int _oub0 =0, _oub1=0, _oub2=0;
        int _Uint = 0; 

        float _C0 = 0, _C1 = 0;  
        float _ou_d_Mv0 = 0;

        //unsigned int long _32Duty;
        int m_SamF = 0;

        m_SamF = (int)(1/m_SamT);

        //decoder
        i_D_error0     = i_D_ref  - m_AdcOut;

        //bo,b1,b2
        _oub0   = (2*m_kp*m_SamF) + m_ki + (2*m_kd * (m_SamF*m_SamF));
        _oub1   = m_ki - (2*m_kp*m_SamF)  - (4*m_kd * (m_SamF*m_SamF));
        _oub2   = 2*m_kd * m_SamF*m_SamF;
        _Uint   = ( _oub0 * i_D_error0 ) +  ( _oub1 * _ou_d_error1 ) + ( _oub2 * _ou_d_error2 );
       
        _C0   = 1 / (float)( 2 * m_AdcR * m_SamF);
        _C1   =  2 * 0.031415926535897932384626433832795028841971;

        _ou_d_Mv0 = _ou_d_Mv1 + ( _C0 * _C1 * (float)_Uint );

        //PWM============================================================================================
        //_32Duty = (m_PWM_DuraionT * _ou_d_Mv0 ) / m_PWM_IOclkT;
        //================================================================================================

        //end
        _ou_d_error2 = _ou_d_error1; 
        _ou_d_error1 = i_D_error0; 

        _ou_d_Mv1 = _ou_d_Mv0;
        return _ou_d_Mv0;   
}




//_____________________________________________________________________________________________________________________








void fcn_Initial( int i_D_ref)
{
   
        _64_s_en1 = 0;
        _64_s_I_error = 0;

        _32_s_en1 = 0;
        _32_s_I_error = 0;

        _ou_s_en1 = 0;
        _ou_s_i_error = 0;


        _64_d_en2 = 0;
        _64_d_en1 = 0;
        _64_d_Mv1 = 0;

        _32_d_en2 = 0;
        _32_d_en1 = 0;
        _32_d_Mv1 = 0;

        _ou_d_error1 = 0;
        _ou_d_error2 = 0;
        _ou_d_Mv1 = 0;
   
}
//_____________________________________________________________________________________________________________________



int main()
{

        //plant
        double yp = 0;
        double Vin = 0;

        double u64 = 0;
        float  u32 = 0;
        float ou32 = 0;

        //=====================================================================================================
        int   AdcOut = 0;
        const int   i_D_ref = 0; 

        const int    AdcR = 1024; //adc 10bit
        const double SamT = 0.001953125 ;
        
        const double Plant_InoutRange = 2 * M_PI;

        //=================================================================================================
        const double f_kp =  1.17; //1.134
        const double f_ki =  0.02; //0.123
        const double f_kd = -0.21; //0.011    //D값을 줄였더니.. 덕 악화가 되었다..

        const int i_kp =  117; //1.134
        const int i_ki =    2; //0.123
        const int i_kd =  -21; //0.011
  
        //===================================================================================================
        const double PWM_DuraionT = 0.0001;
        const double PWM_IOclkT = 0.0001;
  
        fcn_Initial(i_D_ref);

         //||Standard|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||    
  
        /*
        cout << std::endl << "S64 = [ ";
        cout.precision(dou:: max_digits10);
        for( double t1=0; t1 < 1; t1 = t1 + SamT ) 
        {
             
                yp = fcn_plant(t1);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);
        
                u64 = fcn_s_64( Plant_InoutRange, AdcR, SamT, AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT);

                cout << u64 << ",";
        }cout <<  "]";
        */

        cout << std::endl << "%%===========================================================================" << endl;
        cout << std::endl << "S32=[ ";
        cout.precision(flo:: max_digits10);
        for( float t2=0; t2 < 1; t2 = t2 + SamT ) 
        {
                yp = fcn_plant(t2);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);
        
                u32 = fcn_s_32( Plant_InoutRange, AdcR, SamT,AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT);
                cout << u32 << ",";
        }cout <<  "]";



        cout << std::endl << "%%===========================================================================" << endl;
        cout << std::endl << "Sou =[ ";
        cout.precision(flo:: max_digits10);
        for( float t3=0; t3 < 1; t3 = t3 + SamT ) 
        {
             
                yp = fcn_plant(t3);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);
                ou32 = fcn_s_ou(Plant_InoutRange, AdcR, SamT, AdcOut, i_D_ref, i_kp, i_ki, i_kd, PWM_DuraionT, PWM_IOclkT);

                cout << ou32 << ",";
        }cout <<  "]";    

        //||deference|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||    

        /*
        cout << endl << "%%===========================================================================" << endl;
        cout << endl << "D64 = [ ";
        for( double t4=0; t4 < 1; t4 = t4 + SamT ) 
        {
             
                yp = fcn_plant(t4);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);
        
                u64 = fcn_d_64( Plant_InoutRange, AdcR, SamT, AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT);

                //cout.precision(dou:: max_digits10);
                cout << u64 << ",";
        }cout <<  "]";
        */

        cout << endl << "%%===========================================================================" << endl;
        cout << endl << "D32 = [ ";
        for( double t5=0; t5 < 1; t5 = t5 + SamT ) 
        {
             
                yp = fcn_plant(t5);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);
        
                u32 = fcn_d_32( Plant_InoutRange, AdcR, SamT, AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT);

                cout.precision(flo:: max_digits10);
                cout << u32 << ",";
        }cout <<  "]";


        cout << endl << "%%===========================================================================" << endl;
        cout << endl << "Dou =[ ";
        for( double t6=0; t6 < 1; t6 = t6 + SamT ) 
        {
                yp = fcn_plant(t6);
                Vin = fcn_sensor(yp);       
                AdcOut = fcn_adc( Plant_InoutRange, AdcR, Vin);

                u32 = fcn_d_ou(Plant_InoutRange, AdcR, SamT, AdcOut, i_D_ref, i_kp, i_ki, i_kd, PWM_DuraionT, PWM_IOclkT);
                cout.precision(flo:: max_digits10);
                cout << u32 << ",";
        }cout <<  "]";    
      
}



