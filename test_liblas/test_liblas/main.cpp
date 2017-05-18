#include <iostream>
#include <fstream>  // std::ifstream
#include <liblas/liblas.hpp>

int main (int argc, char** argv){
	/*
  std::ifstream ifs;
  ifs.open("D:/t10d.las", std::ios::in | std::ios::binary);

  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  liblas::Header const& header = reader.GetHeader();

  std::cout << "Compressed: " << (header.Compressed() == true) ? "true":"false";
  std::cout << "Signature: " << header.GetFileSignature() << '\n';
  std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';

  while (reader.ReadNextPoint())
  {
      liblas::Point const& p = reader.GetPoint();

      std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() << "\n";
  }*/

	std::cout << "Hello world" << "\n";

}