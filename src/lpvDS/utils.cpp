#include "utils.h"

bool fileUtils::is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}


MatrixXd fileUtils::readMatrix(const char *filename)
{
    int    cols = 0, rows = 0;

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    rows--;

    cout<<"[File Dimensionality] row "<<rows<<" "<<cols<<endl;

    // Populate matrix with numbers.
    MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
}



