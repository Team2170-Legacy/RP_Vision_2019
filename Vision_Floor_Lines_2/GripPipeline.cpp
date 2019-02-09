#include "GripPipeline.h"

namespace grip {

GripPipeline::GripPipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripPipeline::Process(cv::Mat& source0){
        //Step Desaturate0:
        //input
        cv::Mat desaturateInput = source0;
        desaturate(desaturateInput, this->desaturateOutput);
        //Step CV_applyColorMap0:
        //input
        cv::Mat cvApplycolormapSrc = desaturateOutput;
    int cvApplycolormapColormap = cv::COLORMAP_RAINBOW;
        cvApplycolormap(cvApplycolormapSrc, cvApplycolormapColormap, this->cvApplycolormapOutput);
}

/**
 * This method is a generated getter for the output of a Desaturate.
 * @return Mat output from Desaturate.
 */
cv::Mat* GripPipeline::GetDesaturateOutput(){
        return &(this->desaturateOutput);
}
/**
 * This method is a generated getter for the output of a CV_applyColorMap.
 * @return Mat output from CV_applyColorMap.
 */
cv::Mat* GripPipeline::GetCvApplycolormapOutput(){
        return &(this->cvApplycolormapOutput);
}
        /**
         * Converts a color image into shades of grey.
         *
         * @param input The image on which to perform the desaturate.
         * @param output The image in which to store the output.
         */
        void GripPipeline::desaturate(cv::Mat &input, cv::Mat &output) {
                switch (input.channels()) {
                        case 1:
                                // If the input is already one channel, it's already desaturated
                                input.copyTo(output);
                                break;
                        case 3:
                                cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
                                break;
                        case 4:
                                cv::cvtColor(input, output, cv::COLOR_BGRA2GRAY);
                                break;
                        default:
                                throw "Input to desaturate must have 1, 3, or 4 channels";
                }
        }

        /**
         * Applies a color Map to given Image.
         * @param src Image to apply Color Map to.
         * @param colorMap the type of color map to apply.
         * @param dst Output after Color Map is applied.
         */
        void GripPipeline::cvApplycolormap(cv::Mat &src, int colorMap, cv::Mat &dst) {
                cv::applyColorMap(src, dst, colorMap);
        }



} // end grip namespace
