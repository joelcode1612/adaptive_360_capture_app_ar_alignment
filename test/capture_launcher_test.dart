import 'package:adaptive_360_capture_app/main.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('identifies the 26-frame AR route as production', (tester) async {
    await tester.pumpWidget(const MaterialApp(home: CaptureLauncher()));

    expect(find.text('AR capture · Production · 26 frames'), findsOneWidget);
    expect(find.text('Virtual tour · Diagnostic · 30 nodes'), findsOneWidget);
    expect(find.text('Guided flow · Legacy diagnostic'), findsOneWidget);
    expect(
      find.textContaining('Do not use for production AR Capture stations.'),
      findsOneWidget,
    );
  });
}
