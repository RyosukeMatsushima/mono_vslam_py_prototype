import matplotlib.pyplot as plt
import numpy as np

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute

HEADING_SIZE = 0.5

class RouteViewer:

    def __init__(self):
        _, self.ax = plt.subplots(1, 1)

        self.ax.set_xlim(-10.0, 10.0)
        self.ax.set_ylim(-10.0, 10.0)

        self.route_line = []
        self.keyframe_directioen_lines = []
        
    def update(self, keyframes_on_route):

        # remove lines
        [ l.remove() for l in self.route_line ]
        [ l[0].remove() for l in self.keyframe_directioen_lines ]
        self.route_line = []
        self.keyframe_directioen_lines = []

        keyframe_points, keyframe_headings = self.get_points( keyframes_on_route )

        # draw route line
        keyframe_points_x = [ p[0] for p in keyframe_points ]
        keyframe_points_y = [ p[1] for p in keyframe_points ]

        self.route_line = self.ax.plot( keyframe_points_x,
                                        keyframe_points_y,
                                        color='blue' )

        # draw headings
        for point, heading in zip(keyframe_points, keyframe_headings):
            line = self.ax.plot( [ point[0], heading[0] ],
                                 [ point[1], heading[1] ],
                                 color='red' )

            self.keyframe_directioen_lines.append( line )

        plt.draw()
        plt.pause(0.1)


    def get_points(self, keyframes_on_route):

        keyframe_points = []
        keyframe_headings = []

        for keyframe in keyframes_on_route:
            keyframe_yaw = keyframe.keyframe_yaw
            yaw_to_keyframe = keyframe.yaw_to_keyframe
            pixel_distamce = keyframe.pixel_distamce

            keyframe_point = [ pixel_distamce * np.sin( yaw_to_keyframe ),
                               pixel_distamce * np.cos( yaw_to_keyframe ) ]
            keyframe_heading = [ keyframe_point[0] + HEADING_SIZE * np.sin( keyframe_yaw ),
                                  keyframe_point[1] + HEADING_SIZE * np.cos( keyframe_yaw ) ]

            keyframe_points.append( keyframe_point )
            keyframe_headings.append( keyframe_heading )

        return keyframe_points, keyframe_headings



if __name__ == '__main__':

    routeViewer = RouteViewer()
    for i in range(100):
        i = float(i) / 10

        keyframes = []
        for j in range(10):
            keyframeOnRoute = KeyframeOnRoute()
            keyframeOnRoute.keyframe_yaw = i
            keyframeOnRoute.yaw_to_keyframe = i + 0.4
            keyframeOnRoute.pixel_distamce = float(j) / 2 + i
            keyframes.append(keyframeOnRoute)

        routeViewer.update( keyframes )
