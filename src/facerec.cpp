#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>

using namespace cv;
using namespace std;

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';')
{
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) 
    {
        string error_message = "No valid input file";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) 
    {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if (!path.empty() && !classlabel.empty()) 
        {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}


int main()
{
    string fn_haar = "/home/kevin/Documents/opencv-2.4.11/data/haarcascades/haarcascade_frontalface_default.xml";//string(argv[1]);
    string fn_csv = "data/hello.txt";//string(argv[2]);
    int deviceId = 0;//atoi(argv[3]);

    /* testing for what I acutally inputed into the program
    cout << fn_haar << endl;
    cout << fn_csv << endl;
    cout << deviceId << endl;
    int wait;
    cin >> wait;
    return 0;
    */

    // These vectors hold the images and corresponding labels:
    vector<Mat> images;
    vector<int> labels;

    // Read in the data (fails if no valid input filename is given):
    try
    {
        read_csv(fn_csv, images, labels);
    } 
    catch (cv::Exception& e) 
    {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
    }
    // Get the height from the first image
    int im_width = images[0].cols;
    int im_height = images[0].rows;

    CascadeClassifier haar_cascade;
    haar_cascade.load(fn_haar);
    // Get a handle to the Video device:
    VideoCapture cap(deviceId);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 720);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    // Check if we can use this device at all:
    if (!cap.isOpened())
    {
        cerr << "Capture Device ID " << deviceId << "cannot be opened." << endl;
        return -1;
    }
    // Holds the current frame from the Video device:
    Mat frame;
    string prev;
    while (true)
    {
        int humans, faces2;
        vector<string> humanNames;

        ifstream fin ("data/humans.txt"); 
        fin >> humans >> faces2;
        fin.close();

        ifstream fin2 ("data/names.txt");   
        for (int i = 0; i < humans; i++)
        {
            string temp;
            getline(fin2, temp);
            humanNames.push_back(temp);
        }

        fin2.close();

        try
        {
            read_csv(fn_csv, images, labels);
        } 
        catch (cv::Exception& e) 
        {
            cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
            // nothing more we can do
            exit(1);
        }

        // Create a FaceRecognizer and train it on the given  images:
        //Ptr<FaceRecognizer> model_1 = createLBPHFaceRecognizer(1, 8, 8, 8, 600);
        //Ptr<FaceRecognizer> model_2 = createFisherFaceRecognizer(0, 2000.0);
        Ptr<FaceRecognizer> model_3 = createEigenFaceRecognizer(0, 5000.0);
        //model_1->train(images, labels);
        //model_2->train(images, labels);
        model_3->train(images, labels);

        cap >> frame;
        // Clone the current frame:
        Mat original = frame.clone();
        // Convert the current frame to grayscale:
        Mat gray;
        cvtColor(original, gray, CV_BGR2GRAY);
        // Find the faces in the frame:
        vector< Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces, 1.1, 10);

        for(int i = 0; i < faces.size(); i++)
        {
            // Process face by face:
            Rect face_i = faces[i];
            // Crop the face from the image
            Mat face = gray(face_i);

            Mat face_resized;
            cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
            // Now perform the prediction, see how easy that is:
            //int prediction_1 = model_1 -> predict(face_resized);
            //int prediction_2 = model_2 -> predict(face_resized);
            int prediction_3 = model_3 -> predict(face_resized);
           

            //cout << model -> predict(face_resized) << ' ';
            // And finally write all we've found out to the original image!
            // First of all draw a green rectangle around the detected face:
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            // Create the text we will annotate the box with:
            //string box_text = format("%d, %d, %d", prediction_1, prediction_2, prediction_3);
            string box_text = format("Predict: %d", prediction_3);
            // Calculate the position for annotated text (make sure we don't
            // put illegal values in there):
            int pos_x = std::max(face_i.tl().x - 10, 0);
            int pos_y = std::max(face_i.tl().y - 10, 0);
            // And now put it into the image:
            putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

            // Show the result:
            imshow("face_recognizer", original);
            
            char key = (char) waitKey(5);
            if (prediction_3 == -1)
            {
                string human;
                cout << "Please enter your name: ";
                getline(cin, human);
                transform(human.begin(), human.end(), human.begin(), ::tolower);

                if (human.size() == 0)
                    human = prev;
                else
                    prev = human;
                int index = -1;
                for (int i = 0; i < humanNames.size(); i++)
                    if (humanNames[i] == human)
                    {
                        index = i;
                        break;
                    }

                faces2++;
                string str;
                ostringstream temp;
                temp << faces2;
                str = temp.str();
                string output = "data/images/" + human + '_' + str + ".jpg"; 
                imwrite(output, face_resized);

                if (index == -1)
                {
                    string str2;
                    humans++;
                    ostringstream temp2;
                    temp2 << humans - 1;
                    str2 = temp2.str();

                    ofstream fout ("data/names.txt", fstream::app);
                    fout << human << endl;
                    fout.close();

                    ofstream fout2 ("data/hello.txt", fstream::app);
                    fout2 << output + ';' + str2 << endl;
                    fout2.close();
                }
                else
                {
                    string str2;
                    ostringstream temp2;
                    temp2 << index;
                    str2 = temp2.str();
                    ofstream fout2 ("data/hello.txt", fstream::app);
                    fout2 << output + ';' + str2 << endl;
                    fout2.close();
                }

                ofstream fout3 ("data/humans.txt");
                fout3 << humans << ' ' << faces2 << endl;
            }
        

        } 

        // And display it:
        char key = (char) waitKey(5);
        // Exit this loop on escape:
        if (key == 27)
            break;
    }
    return 0;
}