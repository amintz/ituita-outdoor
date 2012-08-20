//
//  ituitaData.h
//  ituita-outdoor
//
//  Created by Felipe Turcheti on 8/15/12.
//  

class ituitaData {
    public:
    
    ituitaData();
    
    int getPersonalNegatives();
    int getPersonalNeutrals();
    int getPersonalPositives();
    
    int getNeighborhoodNegatives();
    int getNeighborhoodNeutrals();
    int getNeighborhoodPositives();
    
    int getCityNegatives();
    int getCityNeutrals();
    int getCityPositives();
    
    void generateRandomValues(int min, int max);
    
    private:
    
    void initDataArrays();
    
    // DATA ARRAYS
    // [0]: negative values
    // [2]: neutral values
    // [2]: positive values
    int personalData[3]; 
    int neighborhoodData[3];
    int cityData[3];   
    
};


