 // liblas  
 #include <liblas/liblas.hpp>  
 #include <liblas/point.hpp>  
 #include <liblas/reader.hpp>  
// #include <liblas/cstdint.hpp>  
 #include <fstream>  
 #include <iostream>  
 #include <string>  
 int main(int argc, char* argv[])  
 {  
   std::ifstream ifs;  
   ifs.open( "c:\\temp\\input.las", std::ios::in | std::ios::binary);   
   liblas::Reader reader(ifs);  
 }  