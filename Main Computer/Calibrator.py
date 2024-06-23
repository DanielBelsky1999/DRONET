from CONFIG import CONFIG_Calibrator

class Calibrator:
    
    # Define normalization parameters and coefficients for each camera and option
    params = [
        # Camera 1, Option 1
        {
            'meanX': 984.2, 'stdX': 581.4, 'meanY': 520, 'stdY': 305.4,
            'coeffs_h': [-0.1730, 27.5188, 0.1813, -0.1848, -0.7262, -0.0515, 1.0201, 0.1306, 0.6783, -0.0608, 0.0689, 0.0282, 0.0305, -0.0493, 0.0081, -0.2361, -0.0852, 0.0410, -0.0418, 0.0369, 0.0347],
            'coeffs_v': [-0.3362, 0.4381, -14.6317, 0.6307, 0.1790, 0.3843, -0.1330, -1.6225, -0.0929, 0.0245, -0.1140, -0.0808, 0.1283, -0.0196, 0.0231, 0.0645, -0.1049, 0.0790, -0.0335, -0.0043, -0.0316]
        },
        # Camera 1, Option 2
        {
            'meanX': 965.3, 'stdX': 523.3, 'meanY': 543.5, 'stdY': 307.1,
            'coeffs_h': [0.0001, 16.8460, -0.0154, 0.0247, -0.2526, 0.0138, -0.1874, -0.0123, 0.0564, -0.0416, -0.0166, 0.0278, -0.0077, 0.0033, -0.0007, -0.0573, 0.0045, 0.0045, 0.0023, -0.0109, 0.0131],
            'coeffs_v': [-0.1205, -0.0889, -9.8248, 0.0597, -0.0177, 0.1040, 0.0269, -0.1725, 0.0045, -0.2693, -0.0240, 0.0111, 0.0059, 0.0056, -0.0104, -0.0069, 0.0361, -0.0023, 0.0096, 0.0054, 0.1923]
        },
        # Camera 1, Option 3
        {
            'meanX': 930, 'stdX': 491.6, 'meanY': 526, 'stdY': 316.7,
            'coeffs_h': [1.3122, 26.6552, 0.0631, -1.3354, -0.6499, -0.1648, 1.1212, 0.1275, 1.0328, 0.0686, 0.0912, 0.0474, -0.0861, -0.0492, 0.0206, -0.2109, -0.0709, 0.0926, -0.0157, 0.0345, -0.0129],
            'coeffs_v': [-0.6375, 0.0833, -17.4182, 0.3331, 0.5577, 0.4372, -0.0092, -1.7771, -0.1286, 0.0234, -0.0072, 0.1271, 0.1552, -0.0255, 0.0177, -0.0033, -0.2079, 0.0525, -0.0731, 0.0167, -0.0342]
        },
        # Camera 2, Option 1
        {
            'meanX': 981.4, 'stdX': 465.8, 'meanY': 515.4, 'stdY': 315.1,
            'coeffs_h': [-0.1633, 22.9908, 0.0358, 0.5681, -0.6565, 0.0253, 0.7432, -0.1270, 0.6355, -0.0489, -0.0648, 0.0237, 0.0541, -0.0867, 0.0193, -0.2012, 0.0287, 0.0603, -0.0111, -0.0064, 0.0121],
            'coeffs_v': [0.5704, 0.0985, -15.9038, 0.2964, -0.0883, 0.5455, 0.0594, -0.9679, 0.0126, 0.0012, -0.0068, -0.0805, 0.0883, -0.0455, -0.0354, -0.0100, -0.0836, 0.0117, -0.0418, 0.0134, 0.0090]
        },
        # Camera 2, Option 2
        {
            'meanX': 960.5, 'stdX': 429.5, 'meanY': 522.3, 'stdY': 289.3,
            'coeffs_h': [0.1286, 13.9378, -0.2104, 0.1318, -0.1348, 0.0007, -0.2267, -0.0517, -0.0579, -0.0389, -0.0223, 0.0259, -0.0147, 0.0010, 0.0062, 0.0203, 0.0136, 0.0240, 0.0104, 0.0122, 0.0113],
            'coeffs_v': [0.4882, -0.3517, -9.5126, 0.0970, -0.0581, 0.0623, 0.0299, -0.0982, 0.0236, 0.1047, -0.0415, 0.0019, -0.0232, 0.0059, 0.0002, -0.0104, 0.0471, -0.0104, -0.0201, -0.0004, -0.0003]
        },
        # Camera 2, Option 3
        {
            'meanX': 937.4, 'stdX': 444.8, 'meanY': 527.8, 'stdY': 305.1,
            'coeffs_h': [-0.2293, 24.6017, -0.4005, 0.8369, -0.5740, 0.1218, 0.5130, -0.0169, 0.7289, -0.1068, -0.0839, -0.0282, 0.0256, -0.0701, -0.0320, -0.0595, 0.0008, 0.1372, 0.0238, 0.0408, 0.0202],
            'coeffs_v': [0.5282, -0.5028, -17.1520, 0.2418, -0.4290, 0.3613, -0.0712, -1.1987, 0.0285, 0.0982, 0.0469, -0.1146, 0.1396, 0.0363, 0.0458, 0.0081, -0.2094, 0.0268, -0.0854, 0.0018, -0.0405]
        }
    ]

    
    def __init__(self):
        camera_num = CONFIG_Calibrator.CAMERA_NUM
        lens_option_num = CONFIG_Calibrator.LENS_NUM
        self.params = Calibrator.params[(camera_num - 1) * 3 + (lens_option_num - 1)]
    
    def Calibrate(self, IGNORED_jetson_id, px1, px2):
        
        # Normalize x and y
        x_norm = (px1 - self.params['meanX']) / self.params['stdX']
        y_norm = (px2 - self.params['meanY']) / self.params['stdY']

        # Calculate the fitted values for horizontal and vertical planes
        fitted_value_horizontal = self._poly55(x_norm, y_norm, self.params['coeffs_h'])
        fitted_value_vertical = self._poly55(x_norm, y_norm, self.params['coeffs_v'])

        # Display the results
        # print(f'Fitted value (horizontal): {fitted_value_horizontal}')
        # print(f'Fitted value (vertical): {fitted_value_vertical}')

        return fitted_value_horizontal, fitted_value_vertical
        
        
        # # # # # # return -10, 30
        # # # # # # # return (jetson_id-1)*20, (jetson_id-1)*20
        # # # # # # # return azimuth_right_deg, elevation_up_deg
        
    
    # Define the polynomial fit function
    def _poly55(self, x, y, coeffs):
        return (coeffs[0] + coeffs[1]*x + coeffs[2]*y + coeffs[3]*x**2 + coeffs[4]*x*y + coeffs[5]*y**2 +
                coeffs[6]*x**3 + coeffs[7]*x**2*y + coeffs[8]*x*y**2 + coeffs[9]*y**3 + coeffs[10]*x**4 +
                coeffs[11]*x**3*y + coeffs[12]*x**2*y**2 + coeffs[13]*x*y**3 + coeffs[14]*y**4 + coeffs[15]*x**5 +
                coeffs[16]*x**4*y + coeffs[17]*x**3*y**2 + coeffs[18]*x**2*y**3 + coeffs[19]*x*y**4 + coeffs[20]*y**5)
                
                
                
if __name__ == "__main__":
    c = Calibrator(1,1)
    
    print(c.Calibrate(0, 1980/2, 1080/2))