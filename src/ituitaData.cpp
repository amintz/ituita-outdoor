//
//  ituitaData.cpp
//  ituita-outdoor
//
//  Created by Felipe Turcheti on 8/15/12.
//

#include "ituitaData.h"
#include "ofMain.h"

ituitaData::ituitaData() {
    initDataArrays();
}

void ituitaData::initDataArrays() {
    for(int i = 0; i < 3; i++) {
        personalData[i] = -1;
        neighborhoodData[i] = -1;
        cityData[i] = -1;
    }    
}

void ituitaData::generateRandomValues(int min, int max) {
    for(int i = 0; i < 3; i++) {
        personalData[i] = ofRandom(min, max);
        neighborhoodData[i] = ofRandom(min, max);
        cityData[i] = ofRandom(min, max);
    }
}

// --------------------------------------------
// MARK: GETTERS FOR PERSONAL DATA

int ituitaData::getPersonalNegatives() {
    return personalData[0];
}
int ituitaData::getPersonalNeutrals() {
    return personalData[1];
}
int ituitaData::getPersonalPositives() {
    return personalData[2];
}

// --------------------------------------------
// MARK: GETTERS FOR NEIGHBOORHOOD DATA

int ituitaData::getNeighborhoodNegatives() {
    return neighborhoodData[0];
}
int ituitaData::getNeighborhoodNeutrals() {
    return neighborhoodData[1];
}
int ituitaData::getNeighborhoodPositives() {
    return neighborhoodData[2];
}

// --------------------------------------------
// MARK: GETTERS FOR CITY DATA

int ituitaData::getCityNegatives() {
    return cityData[0];
}
int ituitaData::getCityNeutrals() {
    return cityData[1];
}
int ituitaData::getCityPositives() {
    return cityData[2];
}


