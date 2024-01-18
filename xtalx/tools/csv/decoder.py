import time


class DateFormat:
    '''
    Takes a time string in *local* time and converts it to a POSIX
    timestamp.
    '''
    def __init__(self, date_fmt='%m/%d/%Y %H:%M:%S'):
        self.date_fmt = date_fmt

    def __call__(self, v):
        return time.mktime(time.strptime(v, self.date_fmt))


class Decoder:
    def __init__(self, f, fmt_table):
        self.fmt_table = fmt_table

        hdr_line = f.readline().strip()
        assert '"' not in hdr_line
        assert "'" not in hdr_line
        self.headers = hdr_line.split(',')
        req_hdrs = set(fmt_table.keys())
        got_hdrs = set(self.headers)
        missing_hdrs = req_hdrs - got_hdrs
        if missing_hdrs:
            raise Exception('Missing columns: %s' % missing_hdrs)

        self.points = []
        for l in f.readlines():
            l = l.strip()
            if not l:
                continue
            if l == hdr_line:
                continue
            if l.startswith('#'):
                continue

            assert '"' not in l
            assert "'" not in l
            fields = l.split(',')
            point  = {}
            for header, field in zip(self.headers, fields):
                fmt = self.fmt_table.get(header)
                if not fmt:
                    continue
                if field == '':
                    point[header] = None
                else:
                    point[header] = fmt(field)
            self.points.append(point)

    def get_column(self, name):
        return [p[name] for p in self.points]
