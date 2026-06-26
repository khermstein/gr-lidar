#ifndef INCLUDED_OFDM_UTILS_H
#define INCLUDED_OFDM_UTILS_H

#include <random>
#include <fstream>
/*
 * Generate the Zadoff-Chu PSS symbols
 */
inline std::vector<gr_complex> gen_zc_symbols(int q, int N){
    std::vector<gr_complex> x(N);
    const double pi = std::acos(-1.0);
    double phi;
    for(int i=0; i<N; i++){
        phi = (-1 * pi * q * i * (i+1)) / N;
        //mult by sqrt(2) since we only occupy 1/2 of the subcarriers
        x[i] = gr_complex(static_cast<float>(0.5*1.414*std::cos(phi)),
                        static_cast<float>(0.5*-1.414*std::sin(phi)));
        
    }
    return x;
}

/*
 * Conjugate and reflect an existing complex data sequence
 */
inline void conjugate(std::vector<gr_complex>& data){
    gr_complex j(0.0f, 1.0f);  // Represents 'i'
    for (unsigned int i = 0; i < data.size(); ++i) {
        data[i] = std::conj(data[i]) * j;  // Second pilot
    }
}

//Generate a random sequence of +1, -1 to use as pilot symbols
//We use the same pilot symbols for each packet
//By setting the seed we can recreate the symbols at the rx
inline std::vector<gr_complex> gen_pilot_symbols(int seed, int length){
    //code from Gemini
    std::mt19937 gen(seed); // Standard mersenne_twister_engine
    std::uniform_int_distribution<> dis(0, 1); // 0 or 1
    
    std::vector<gr_complex> pilots;
    pilots.reserve(length);
    float val;
    for (int i = 0; i < length; ++i) {
        // Map 0 -> -1.0 and 1 -> 1.0
        val = (dis(gen) == 0) ? -1.0f : 1.0f;
        pilots.push_back(gr_complex(val, 0.0f));
    }
    return pilots;
}

/*
 * Place the symbols associated with corresponding carriers onto the proper indices of the frame
 * TODO: Make memory save with packet size checks
 */
inline void allocate_onto_carriers(std::vector<gr_complex>& packet, const gr_complex* symbols, int num_symbols, const std::vector<int>& carriers, int fft_len){
    int symbol_idx=0;
    int carrier_idx;
    //Place symbols on carriers with first symbol on first positive frequency
    for(int carrier : carriers){
        carrier_idx = (carrier < 0) ? (fft_len + carrier) : carrier;
        packet[carrier_idx] = symbols[symbol_idx++];
    }
}

inline void allocate_onto_carriers(gr_complex* packet, const gr_complex* symbols, int num_symbols, const std::vector<int>& carriers, int fft_len){
    int symbol_idx=0;
    int carrier_idx;
    //Place symbols on carriers with first symbol on first positive frequency
    for(int carrier : carriers){
        carrier_idx = (carrier < 0) ? (fft_len + carrier) : carrier;
        packet[carrier_idx] = symbols[symbol_idx++];
    }
}

inline void writeComplexBinary(const std::string& filename, const std::vector<gr_complex>& data) {
    // Open the file in output and binary modes
    std::ofstream outFile(filename, std::ios::out | std::ios::binary);

    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the raw data to the file
    // The data() member function of vector returns a pointer to the first element's memory location
    // reinterpret_cast<const char*> is used to treat this memory as a sequence of bytes
    outFile.write(
        reinterpret_cast<const char*>(data.data()),
        data.size() * sizeof(gr_complex) // Calculate total bytes to write
    );

    // Close the file stream
    outFile.close();

    std::cout << "Successfully wrote " << data.size() << " complex floats to " << filename << std::endl;
}

inline void writeBinary(const std::string& filename, const std::vector<unsigned char>& data, bool append) {
    // Open the file in output and binary modes
    std::ofstream outFile;

    if(append){
        outFile.open(filename, std::ios::out | std::ios::binary | std::ios::app);
    } else {
        outFile.open(filename, std::ios::out | std::ios::binary);
    }

    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the raw data to the file
    // The data() member function of vector returns a pointer to the first element's memory location
    // reinterpret_cast<const char*> is used to treat this memory as a sequence of bytes
    outFile.write(
        reinterpret_cast<const char*>(data.data()),
        data.size() * sizeof(unsigned char) // Calculate total bytes to write
    );

    // Close the file stream
    outFile.close();

    //std::cout << "Successfully wrote " << data.size() << " bytes to " << filename << std::endl;
}

//LiDAR data stuff
struct PointXYZ {
    double x;
    double y;
    double z;
    uint8_t reflectivity;
};


#endif /* INCLUDED_OFDM_UTILS_H */
