#!/usr/bin/env python

'''
A module to construct corpora databases for the Hyperscan benchmarker
(hsbench).

After construction, simply add blocks with the add_chunk() method, then call
finish() when you're done.
'''

import os.path

try:
    from sqlite3 import dbapi2 as sqlite
except:
    from pysqlite2 import dbapi2 as sqlite

class CorpusBuilder:
    SCHEMA = '''
CREATE TABLE chunk (
    id integer primary key,
    stream_id integer not null,
    data blob
);
'''

    def __init__(self, outfile):
        if os.path.exists(outfile):
            raise RuntimeError("Database '%s' already exists" % outfile)
        self.outfile = outfile
        self.db = sqlite.connect(self.outfile)
        self.db.executescript(CorpusBuilder.SCHEMA)
        self.current_chunk_id = 0;

    def add_chunk(self, stream_id, data):
        chunk_id = self.current_chunk_id;
        c = self.db.cursor()
        q = 'insert into chunk (id, stream_id, data) values (?, ?, ?)'
        c.execute(q, (chunk_id, stream_id, sqlite.Binary(data)))
        self.current_chunk_id += 1
        return chunk_id

    def finish(self):
        self.db.commit()

        c = self.db.cursor()
        q = 'create index chunk_stream_id_idx on chunk(stream_id)'
        c.execute(q)

        c = self.db.cursor()
        q = 'vacuum'
        c.execute(q)

        c = self.db.cursor()
        q = 'analyze'
        c.execute(q)

        self.db.commit()
