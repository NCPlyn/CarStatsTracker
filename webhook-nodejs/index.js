var express = require('express'),
    app = express(),
    PORT = 80;

app.use(express.json());

app.post('/uplink', function (req, res) {
	var jsonData = JSON.stringify(req.body);
	var jsonParsed = JSON.parse(jsonData);
	var buf = Buffer.from(jsonParsed.uplink_message.frm_payload, 'base64');
	var str = buf.toString('utf-8');
	console.log(str);
    res.status(200).end()
});

app.listen(PORT, () => console.log(`Server running on port ${PORT}`))