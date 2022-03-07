def parse_nmea(file):

    # just a sample for GPRMC + hdt merged file the function can be refined further to use original log
    with open("./lat_longs_merged.txt",'r') as f:
        lines = f.readlines()
    lines_gps = [line.split("\t")[1] for line in lines] 
    lines_hdt = [float(line.split("\t")[0].split(",")[1].strip()) for line in lines if not (line.split("\t")[0].split(",")[1].strip() == "") ] 
    
    print(lines_gps[0].split()[0])
    lat = [str(i.split()[0].strip()) for i in lines_gps]
    longi = [str(i.split()[1].strip()) for i in lines_gps]

    lat_conv = [float(l[:2]) + float(l[2:])/60.0 for l in lat]
    longi_conv = [float(l[:3]) + float(l[3:])/60.0 for l in longi]

    # convert true heading to enu
    # heading = np.unwrap([0,-(heading - pi/2)])[1]

    return lat_conv, longi_conv, lines_hdt
