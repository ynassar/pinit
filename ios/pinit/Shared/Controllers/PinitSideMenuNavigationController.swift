import UIKit

class PinitSideMenuNavigationController : PinitNavigationController {
    
    var buttonTest: UIButton!
    
    var shit: UIBarButtonItem!
    
    override func viewDidLoad() {
        buttonTest = UIButton(frame: CGRect.zero)
        super.viewDidLoad()
 
        let profileMenuButton = UIButton(frame: CGRect.zero)
        profileMenuButton.setImage(UIImage(named: "menuIcon"), for: .normal)
        profileMenuButton.addTarget(self, action: #selector(menuButtonClicked), for: .touchDown)
        shit = UIBarButtonItem(customView: profileMenuButton)


        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
            UIBarButtonItem(customView: profileMenuButton)
        ]
        
        self.view.addSubview(buttonTest)
        buttonTest.backgroundColor = .white
        
        self.view.backgroundColor = .red
        
        buttonTest = buttonTest
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.1)
            .setConstraintWithConstant(selfAttribute: .top, relativeView: self.view, relativeAttribute: .top, constant: 10)
        
        buttonTest.addTarget(self, action: #selector(menuButtonClicked), for: .touchUpInside)

    }
    
    @objc public func menuButtonClicked() {
        print("whattt")
    }
    
}
