#include <iostream>
#include <map>
#include <string>
#include <fstream>
#include <algorithm>

class ConfigReader{
    //---------------------------------------------------------------------------
    // The configurator is a simple map string (key, value) pairs.
    // The file is stored as a simple listing of those pairs, one per line.
    // The key is separated from the value by an equal sign '='.
    // Commentary begins with the first non-space character on the line a hash or
    // semi-colon ('#' or ';').
    //
    // Notice that the configuration file format does not permit values to span
    // more than one line, commentary at the end of a line. 
public:
    ConfigReader(){}
    ConfigReader(std::string filename){
        std::ifstream ifs(filename, std::ifstream::in);
        if (!ifs)
        {
            error_ = true;
            std::cout << "Error: Can't open the file" << std::endl;
            ifs.close();
        }
        else{
            ifs >> *this;
            ifs.close();
        }
        
    }
    ~ConfigReader(){}

private:
    bool error_ = false;
    std::map <std::string, std::string> data_;
public:
    bool CheckError(){
        return error_;
    }

    std::map <std::string, std::string> GetAllData(){
        return data_;
    }

    std::string Get(std::string name, std::string default_value){
        if (data_.count(name)){
            return data_[name];
        }
        else{
            std::cout << "Failed to load data ''" << name << "''" << std::endl;
            return default_value;
        }
    }

    double GetReal(std::string name, double default_value){
        std::string valstr = Get(name, "");
        if (valstr == ""){
            return default_value; 
        }
        char* end = 0;
        double val = strtod(valstr.c_str(), &end);
        if (end > valstr.c_str() && val != HUGE_VAL){
            return val;
        }
        else{
            std::cout << "Failed to load data ''" << name << "''" << std::endl;
            return default_value; 
        }
    }

    int GetInteger(std::string name, int default_value){
        std::string valstr = Get(name, "");
        if (valstr == ""){
            return default_value; 
        }
        char* end = 0;
        int val = strtol(valstr.c_str(), &end, 0);
        if (end != valstr.c_str() && val != HUGE_VAL){
            return val;
        }
        else{
            std::cout << "Failed to load data ''" << name << "''" << std::endl;
            return default_value; 
        }
    }

    bool GetBoolean(std::string name, bool default_value){
        std::string valstr = Get(name, "");
        if (valstr == ""){
            return default_value; 
        }
        // Convert to lower case to make string comparisons case-insensitive
        std::transform(valstr.begin(), valstr.end(), valstr.begin(), ::tolower);
        if (valstr == "true" || valstr == "yes" || valstr == "on" || valstr == "1"){
            return true;
        }
        else if (valstr == "false" || valstr == "no" || valstr == "off" || valstr == "0"){
            return false;
        }
        else{
            std::cout << "Failed to load data ''" << name << "''" << std::endl;
            return default_value;
        } 
    }


    //---------------------------------------------------------------------------
    // The extraction operator reads configuration until EOF.
    // Invalid data is ignored.
    friend std::istream& operator >> (std::istream& ins, ConfigReader& config_reader){
        std::string s, key, value;
        
        // For each (key, value) pair in the file
        while (std::getline(ins, s)){
            std::string::size_type begin = s.find_first_not_of(" \f\t\v");
            
            // Skip blank lines
            if (begin == std::string::npos) continue;
    
            // Skip commentary
            if (std::string("#;").find(s[begin]) != std::string::npos) continue;
    
            // Extract the key value
            std::string::size_type end = s.find('=', begin);
            key = s.substr(begin, end - begin);
    
            // (No leading or trailing whitespace allowed)
            key.erase(key.find_last_not_of(" \f\t\v" ) + 1);
    
            // No blank keys allowed
            if (key.empty()) continue;
    
            // Extract the value (no leading or trailing whitespace allowed)
            begin = s.find_first_not_of(" \f\n\r\t\v", end + 1);
            end   = s.find_last_not_of(" \f\n\r\t\v") + 1;
            value = s.substr(begin, end - begin);
    
            // Insert the properly extracted (key, value) pair into the map
            config_reader.data_[key] = value;
        }
        return ins;
    };

    //---------------------------------------------------------------------------
    // The insertion operator writes all config_reader data to stream.
    friend std::ostream& operator << (std::ostream& outs, const ConfigReader& config_reader){
        std::map<std::string, std::string>::const_iterator iter;
        for (iter = config_reader.data_.begin(); iter != config_reader.data_.end(); iter++){
            std::cout << iter->first << " = " << iter->second << std::endl;
        }
        return outs;
    };
};
