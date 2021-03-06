class CoordonneeGps {
    double m_latitudeDeg;
    double m_longitudeDeg;
};


class GpsReader {
    int m_bufferIndLast;
    char m_buffer[200];
    
    int m_tempInd;
    void parseBuffer();
    void readUntilCommat();
    void error();
    void debug();
    double readDouble();
    int getOneInt();
    



public:
    double m_lastLatitudeDeg;
    double m_lastLongitudeDeg;
    int m_lastFix;
    
    GpsReader();

    void resetBuffer();
    bool readChar(char c);
    void readNextFrame();
    
    static double convertDegMinToDecDeg (float degMin);

};
