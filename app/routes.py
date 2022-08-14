#!/usr/bin/env python

from flask import Flask, render_template, request
import logging

import route_tracker

app = Flask(__name__, static_folder='route_tracker_movies')

@app.route('/track', methods=['GET'])
def show():
    app.logger.info("show start")
    video_src = request.args.get('video', 0)
    route_dir = request.args.get('route', './route_imgs/')
    
    route_tracker.App(video_src, route_dir).run(web=True)
    return render_template('show.html', video_url='route_tracker_movies/output.webm')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8000, threaded=True)

