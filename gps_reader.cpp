#include "gps_reader.h"
#include "WProgram.h"

GpsReader::GpsReader(){
    resetBuffer();
}

void GpsReader::resetBuffer(){
    m_bufferIndLast = 0;
}

bool GpsReader::readChar(char c){
    if(c == '$'){
        Serial.printf("readChar $\n");
        resetBuffer();
    } else if(c == '\n'){
        Serial.printf("readChar \\n\n");
        parseBuffer();
        return true;
    } else {
        this->m_buffer[m_bufferIndLast] = c;
        m_bufferIndLast++;
    }
    return false;
}

void GpsReader::error(){
    Serial.print("error");
}

void GpsReader::debug(){
    Serial.print(m_tempInd);
    Serial.print(" ");
    Serial.print(m_buffer[m_tempInd-1]);
    Serial.print(m_buffer[m_tempInd]);
    Serial.print(m_buffer[m_tempInd+1]);
    Serial.print("\n");
}

void GpsReader::readUntilCommat(){
    while(m_tempInd < m_bufferIndLast){
        if(m_buffer[m_tempInd] == ','){
            ++m_tempInd;
            return;
        }
        ++m_tempInd;
    }
    error();
}

double GpsReader::readDouble(){
    double res = 0;
    double virgule_part = 1;
    bool virgule = false;
    while(m_tempInd < m_bufferIndLast){
        char c = m_buffer[m_tempInd];
        int number = 0;
        if(c == ','){
            ++m_tempInd;
            return res;
        } else if(c =='0'){
            number = 0;
        } else if(c =='1'){
            number = 1;
        } else if(c =='2'){
            number = 2;
        } else if(c =='3'){
            number = 3;
        } else if(c =='4'){
            number = 4;
        } else if(c =='5'){
            number = 5;
        } else if(c =='6'){
            number = 6;
        } else if(c =='7'){
            number = 7;
        } else if(c =='8'){
            number = 8;
        } else if(c =='9'){
            number = 9;
        } else if(c =='.'){
            virgule = true;
        }
        if(!virgule){
            res = res * 10 + number;
        } else {

            res = res + number * virgule_part;
            virgule_part = virgule_part * 0.1;
        }
        ++m_tempInd;
    }
    error();
}

//$GPGGA,114608.00,4905.46094,N,00332.09303,E,2,07,1.46,87.8,M,46.3,M,,0000*6B
void GpsReader::parseBuffer(){
    for(int i = 0; i < m_bufferIndLast; ++i){
        Serial.print(m_buffer[i]);
    }
    Serial.print("\n");
    m_tempInd = 0;
    readUntilCommat();
    readUntilCommat();
    //debug();
    m_lastLatitudeDeg = readDouble();
    readUntilCommat();
    //debug();
    m_lastLongitudeDeg = readDouble();
    
}

void GpsReader::readNextFrame(){
    while(true){
        while ( Serial2.available()){
            char c = Serial2.read();
            if(this->readChar(c)){
                return;
            }
        }
    }
}

double GpsReader::convertDegMinToDecDeg (float degMin)
{
	// Serial.println(degMin);
	int h = degMin/100;
	int minu = (degMin-(h*100));
	float sec =(degMin-((h*100)+minu))*100.0;
	float minum = minu +(sec/60.0);
	float decDeg = h + (minum/60.0) ;
	// Serial.println(decDeg,6);
	return decDeg;
}


